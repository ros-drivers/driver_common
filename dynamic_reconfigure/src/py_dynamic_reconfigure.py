#! /usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

from __future__ import with_statement

import roslib; roslib.load_manifest('dynamic_reconfigure')
import rospy
import rosservice                  
import threading
import time
from dynamic_reconfigure.srv import Reconfigure as ReconfigureSrv
from dynamic_reconfigure.msg import Config as ConfigMsg
from dynamic_reconfigure.msg import ConfigDescription as ConfigDescrMsg
from dynamic_reconfigure.msg import IntParameter, BoolParameter, StrParameter, DoubleParameter, ParamDescription

def find_reconfigure_services():
    return [s[:-len('/set_parameters')] for s in rosservice.get_service_list() if s.endswith('/set_parameters')] 

def get_parameter_names(descr):
    return descr.defaults.keys()

def _encode_config(config):
    msg = ConfigMsg()
    for k, v in config.items():
        ## @todo add more checks here?
        if   type(v) == int:   msg.ints.append(IntParameter(k, v))
        elif type(v) == bool:  msg.bools.append(BoolParameter(k, v))
        elif type(v) == str:   msg.strs.append(StrParameter(k, v))
        elif type(v) == float: msg.doubles.append(DoubleParameter(k, v))
    return msg

def _decode_config(msg):
    return dict([(kv.name, kv.value) for kv in msg.bools + msg.ints + msg.strs + msg.doubles])

def _encode_description(descr):
    msg = ConfigDescrMsg()
    msg.max = _encode_config(descr.max)
    msg.min = _encode_config(descr.min)
    msg.dflt = _encode_config(descr.defaults)
    for param in descr.config_description:
        msg.parameters.append(ParamDescription(param['name'], param['type'], param['level'], param['description'], param['edit_method']))
    return msg

def _decode_description(msg):
    descr = []
    mins = _decode_config(msg.min)
    maxes = _decode_config(msg.max)
    defaults = _decode_config(msg.dflt)
    for param in msg.parameters:
        name = param.name
        descr.append({
            'name': name,
            'min': mins[name],
            'max': maxes[name],
            'default': defaults[name],
            'type': param.type,
            'description': param.description,
            'edit_method': param.edit_method,
            })
    return descr

class DynamicReconfigureClient(object):
    """
    python dynamic_reconfigure client API
    """
    def __init__(self, name, timeout=None, config_callback=None, description_callback=None):
        """
        Connect to dynamic_reconfigure server and return a client object
        
        @param name: name of the server to connect to (usually the node name)
        @type  str
        @param timeout: time to wait before giving up
        @type float
        @param config_callback: callback for server parameter changes
        @param description_callback: internal use only as the API has not stabilized
        @return a client object
        @rtype DynamicReconfigureClient
        """
        self.name              = name
        self.config            = None
        self.param_description = None
        
        self._param_types = None

        self._cv = threading.Condition()

        self._config_callback      = config_callback
        self._description_callback = description_callback

        self._set_service      = self._get_service_proxy('set_parameters', timeout)       
        self._updates_sub      = self._get_subscriber('parameter_updates',      ConfigMsg,      self._updates_msg)
        self._descriptions_sub = self._get_subscriber('parameter_descriptions', ConfigDescrMsg, self._descriptions_msg)

    def get_configuration(self, timeout=None):
        """
        Return the latest received server configuration (wait to receive
        one if none have been received)

        @param timeout: time to wait before giving up
        @type float
        @returns dictionary mapping parameter names to values
        @rtype dict of (str, value) pairs
        """
        if timeout is None:
            with self._cv:
                while self.config is None:
                    if rospy.is_shutdown():
                        return None
                    self._cv.wait()
        else:
            start_time = time.time()      
            with self._cv:
                while self.config is None:
                    if rospy.is_shutdown():
                        return None
                    secs_left = timeout - (time.time() - start_time)
                    if secs_left <= 0.0:
                        break
                    self._cv.wait(secs_left)

        return self.config

    def get_parameter_descriptions(self, timeout=None):
        """
        Return a description of the parameters for the server.
        Please do not use this method as the type that is returned may
        change.
        
        @param timeout: time to wait before giving up
        @type float
        """
        if timeout is None:
            with self._cv:
                while self.param_description is None:
                    if rospy.is_shutdown():
                        return None
                    self._cv.wait()
        else:
            start_time = time.time()
            with self._cv:
                while self.param_description is None:
                    if rospy.is_shutdown():
                        return None
                    secs_left = timeout - (time.time() - start_time)
                    if secs_left <= 0.0:
                        break
                    self._cv.wait(secs_left)

        return self.param_description

    def update_configuration(self, changes):
        """
        Change the server's configuration

        @param changes: dictionary of key value pairs for the parameters that are changing
        @type dict of (str, value) pairs
        """
        # Retrieve the parameter descriptions
        if self.param_description is None:
            self.get_parameter_descriptions()

        # Cast the parameters to the appropriate types
        if self.param_description is not None:
            for name, value in changes.items()[:]:
                dest_type = self._param_types.get(name)
                if dest_type is None:
                    raise Exception('don\'t know type for parameter: %s' % name)
                
                changes[name] = dest_type(value)
        
        config = _encode_config(changes)
        msg    = self._set_service(config).config
        resp   = _decode_config(msg)

        return resp

    def close(self):
        """
        Close connections to the server
        """
        self._updates_sub.unregister()
        self._descriptions_sub.unregister()

    ## config_callback

    def get_config_callback(self):
        """
        Retrieve the config_callback
        """
        return self._config_callback

    def set_config_callback(self, value):
        """
        Set the config_callback
        """
        self._config_callback = value
        if self._config_callback is not None:
            self._config_callback(self.config)

    config_callback = property(get_config_callback, set_config_callback)

    ## description_callback        

    def get_description_callback(self):
        """
        Get the current description_callback
        """
        return self._config_callback

    def set_description_callback(self, value):
        """
        Set the description callback. Do not use as the type of the
        description callback may change.
        """
        self._description_callback = value
        if self._description_callback is not None:
            self._description_callback(self.param_description)

    description_callback = property(get_description_callback, set_description_callback)

    # Implementation

    def _get_service_proxy(self, suffix, timeout):
        service_name = rospy.resolve_name(self.name + '/' + suffix)
        rospy.wait_for_service(service_name, timeout)
        return rospy.ServiceProxy(service_name, ReconfigureSrv)
    
    def _get_subscriber(self, suffix, type, callback):
        topic_name = rospy.resolve_name(self.name + '/' + suffix)
        return rospy.Subscriber(topic_name, type, callback=callback)

    def _updates_msg(self, msg):
        self.config = _decode_config(msg)
        
        with self._cv:
            self._cv.notifyAll()
        if self._config_callback is not None:
            self._config_callback(self.config)

    def _descriptions_msg(self, msg):
        self.param_description = _decode_description(msg)

        # Build map from parameter name to type
        self._param_types = {}
        for p in self.param_description:
            n, t = p.get('name'), p.get('type')
            if n is not None and t is not None:
                self._param_types[n] = self._param_type_from_string(t)

        with self._cv:
            self._cv.notifyAll()
        if self._description_callback is not None:
            self._description_callback(self.param_description)

    def _param_type_from_string(self, type_str):
        if   type_str == 'int':    return int
        elif type_str == 'double': return float
        elif type_str == 'str':    return str
        elif type_str == 'bool':   return bool
        else:
            raise Exception('Parameter has unknown type: %s. This is a bug in dynamic_reconfigure.' % type_str)

class DynamicReconfigureServer(object):
    def __init__(self, type, callback):
        self.mutex = threading.Lock()
        self.type = type
        self.config = type.defaults
        self.description = _encode_description(type)
        self._copy_from_parameter_server()
        self.callback = callback
        self._clamp(self.config) 

        self.descr_topic = rospy.Publisher('~parameter_descriptions', ConfigDescrMsg, latch=True)
        self.descr_topic.publish(self.description);
        
        self.update_topic = rospy.Publisher('~parameter_updates', ConfigMsg, latch=True)
        self._change_config(self.config, type.all_level)
        
        self.set_service = rospy.Service('~set_parameters', ReconfigureSrv, self._set_callback)

    def _copy_from_parameter_server(self):
        for param in self.type.config_description:
            try:
                self.config[param['name']] = rospy.get_param("~" + param['name'])
            except KeyError:
                pass

    def _copy_to_parameter_server(self):
        for param in self.type.config_description:
            rospy.set_param("~" + param['name'], self.config[param['name']])

    def _change_config(self, config, level):
        self.config = self.callback(config, level)
        if self.config is None:
            msg = "Reconfigure callback should return a possibly updated configuration."
            rospy.logerr(msg)
            raise Exception(msg)
        
        self._copy_to_parameter_server()
        
        self.update_topic.publish(_encode_config(config))

        return self.config
   
    def _calc_level(self, config1, config2):
        level = 0
        for param in self.type.config_description:
            if config1[param['name']] != config2[param['name']]:
                level |= param['level']

        return level

    def _clamp(self, config):
        for param in self.type.config_description: 
            maxval = self.type.max[param['name']] 
            minval = self.type.min[param['name']] 
            val = config[param['name']]
            if val > maxval and maxval != "": 
                config[param['name']] = maxval 
            elif val < minval and minval != "": 
                config[param['name']] = minval 

    def _set_callback(self, req):
        return _encode_config(self.update_configuration(_decode_config(req.config)))

    def update_configuration(self, changes):
        with self.mutex:
            new_config = dict(self.config)
            new_config.update(changes)
            self._clamp(new_config)
            return self._change_config(new_config, self._calc_level(new_config, self.config))
