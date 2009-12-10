#! /usr/bin/env python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

from __future__ import with_statement

import roslib; roslib.load_manifest('dynamic_reconfigure')
import rospy
import rosservice                  
import threading
from dynamic_reconfigure.srv import Reconfigure as ReconfigureSrv
from dynamic_reconfigure.msg import Config as ConfigMsg
from dynamic_reconfigure.msg import ConfigDescription as ConfigDescrMsg
from dynamic_reconfigure.msg import IntParameter, BoolParameter, StrParameter, DoubleParameter, ParamDescription

def find_reconfigure_services():
    services = rosservice.get_service_list()
    found = []
    for i in range(0, len(services)):
        ## @todo This check can be fooled, but will be sufficient for now.
        if services[i].endswith('/set_parameters'):
            #print services[i], rosservice.get_service_type(services[i])
            found.append(services[i][0:-15])
    return found

def get_parameter_names(type):
    return type.defaults.keys()

def _encode_config(dict):
    config = ConfigMsg()
    for k,v in dict.items():
        ## @todo add more checks here?
        if type(v) == int:
            config.ints.append(IntParameter(k, v))
        elif type(v) == bool:
            config.bools.append(BoolParameter(k, v))
        elif type(v) == str:
            config.strs.append(StrParameter(k, v))
        elif type(v) == float:
            config.doubles.append(DoubleParameter(k, v))
    return config

def _decode_config(config_encoded):
    config = {}
    for kv in config_encoded.bools + config_encoded.ints + config_encoded.strs + config_encoded.doubles:
        config[kv.name] = kv.value
    return config

def _make_description(type):
    descr = ConfigDescrMsg()
    descr.max = _encode_config(type.max)
    descr.min = _encode_config(type.min)
    descr.dflt = _encode_config(type.defaults)
    for param in type.config_description:
        descr.parameters.append(ParamDescription(
            param['name'], param['type'], param['level'], param['description'], param['edit_method']))
    return descr

class DynamicReconfigureClient:
    def __init__(self, name, timeout = None, config_callback = None,
            description_callback = None):
        self.name = name
        self.set_service = self._get_service_proxy('set_parameters', timeout)
        self.config = None
        self.param_description = None
        self.config_callback = config_callback
        self.description_callback = description_callback
        self.cv = threading.Condition();
        self.description_callback_sub = self._get_subscriber('parameter_descriptions', 
                        ConfigDescrMsg, self._description_callback)
        self.change_callback_sub = self._get_subscriber('parameter_updates', 
                        ConfigMsg, callback=self._parameter_callback)
    
    def get_configuration(self, wait=True):
        self.cv.acquire()
        while self.config == None:
            self.cv.wait()
        self.cv.release()
        return self.config

    def get_parameter_descriptions(self, wait=True):
        self.cv.acquire()
        while self.param_description == None:
            self.cv.wait()
        self.cv.release()
        return self.param_description

    def update_configuration(self, dict):
        config = _encode_config(dict)
        resp = self.set_service(config).config
        return _decode_config(resp)
    
    def _get_subscriber(self, suffix, type, callback):
        topic_name = rospy.resolve_name(self.name + '/' + suffix)
        return rospy.Subscriber(topic_name, type, callback = callback)

    def _get_service_proxy(self, suffix, timeout):
        service_name = rospy.resolve_name(self.name + '/' + suffix)
        #print "waiting for service", service_name
        rospy.wait_for_service(service_name, timeout)
        #print service_name
        #service_class = rosservice.get_service_class_by_name(service_name)
        #print service_class
        return rospy.ServiceProxy(service_name, ReconfigureSrv)#, service_class

    def _parameter_callback(self, msg):
        #print "Parameter callback"
        self.config = _decode_config(msg)
        self.cv.acquire()
        self.cv.notifyAll()
        self.cv.release()
        if self.config_callback:
            self.config_callback(descr)

    def _description_callback(self, msg):
        #print "Description callback"
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
        self.param_description = descr
        self.cv.acquire()
        self.cv.notifyAll()
        self.cv.release()
        if self.description_callback:
            self.description_callback(descr)

class DynamicReconfigureServer:
    def __init__(self, type, callback):
        self.mutex = threading.Lock()
        with self.mutex:
            self.type = type
            self.config = type.defaults
            self.description = _make_description(type)
            self._copy_from_parameter_server()
            self.callback = callback
            self._clamp(self.config) 
            self.descr_topic = rospy.Publisher('~parameter_descriptions',ConfigDescrMsg, latch=True)
            self.descr_topic.publish(self.description);
            self.update_topic = rospy.Publisher('~parameter_updates',ConfigMsg,latch=True)
            self._change_config(self.config, type.all_level)
            self.set_service = rospy.Service('~set_parameters', ReconfigureSrv, self._set_callback)

    def _copy_from_parameter_server(self):
        for param in self.type.config_description:
            try:
                self.config[param['name']] = rospy.get_param("~"+param['name'])
            except KeyError:
                pass

    def _copy_to_parameter_server(self):
        for param in self.type.config_description:
            rospy.set_param("~"+param['name'], self.config[param['name']])

    def _change_config(self, config, level):
        self.update_topic.publish(_encode_config(config))
        self.config = self.callback(config, level)
        if self.config == None:
            msg = "Reconfigure callback should return a possibly updated configuration."
            rospy.logerr(msg)
            raise Exception(msg)
        self._copy_to_parameter_server()
        return self.config
   
    def _calc_level(self, config1, config2):
        level = 0
        for param in self.type.config_description:
            if config1[param['name']] != config2[param['name']]:
                level = level | param['level']
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
        with self.mutex:
            new_config = dict(self.config)
            new_config.update(_decode_config(req.config))
            self._clamp(new_config)
            return _encode_config(self._change_config(new_config, self._calc_level(new_config, self.config)))


