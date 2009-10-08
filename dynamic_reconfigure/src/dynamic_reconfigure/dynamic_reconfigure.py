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


import roslib; roslib.load_manifest('dynamic_reconfigure')
import rospy
import rosservice

def find_reconfigure_services():
    services = rosservice.get_service_list()
    getters = set()
    setters = set()
    for i in range(0, len(services)):
        ## @todo This check can be fooled, but will be sufficient for now.
        if services[i].endswith('/get_configuration'):
            getters.add(services[i][0:-18])
        if services[i].endswith('/set_configuration'):
            setters.add(services[i][0:-18])
    return list(getters & setters)

class DynamicReconfigureClient:
    def __init__(self, name, timeout = None):
        self.name = name
        (self.get_service, self.get_service_class) = self.get_service_proxy('get_configuration', timeout)
        (self.set_service, self.set_service_class) = self.get_service_proxy('set_configuration', timeout)
    
    def get_service_proxy(self, suffix, timeout):
        service_name = rospy.resolve_name(self.name + '/' + suffix)
        #print "waiting for service", service_name
        rospy.wait_for_service(service_name, timeout)
        #print service_name
        service_class = rosservice.get_service_class_by_name(service_name)
        #print service_class
        return rospy.ServiceProxy(service_name, service_class), service_class

    def get_configuration(self):
        return self.get_service()

    def update_configuration(self, dict):
        config = self.get_configuration().config
        #print
        #print dir(config.config)
        #config.__dict__[name] = value
        for k,v in dict.items():
            config.__setattr__(k, v)
        #print config
        req = self.set_service_class._request_class(config)
        resp = self.set_service(req).config
        return resp

class DynamicReconfigureServer:
    def __init__(self, type, callback):
        self.type = type
        self.config = type.defaults
        self.copy_from_parameter_server()
        self.callback = callback
        self.get_service = rospy.Service('~get_configuration', type.GetClass, self.get_callback)
        self.set_service = rospy.Service('~set_configuration', type.SetClass, self.set_callback)
        self.clamp(self.config) 
        self.change_config(self.config, type.all_level)

    def copy_from_parameter_server(self):
        for param in self.type.config_description:
            try:
                self.config.__setattr__(param['name'],rospy.get_param("~"+param['name']))
            except KeyError:
                pass

    def copy_to_parameter_server(self):
        for param in self.type.config_description:
            rospy.set_param("~"+param['name'], self.config.__getattribute__(param['name']))

    def change_config(self, config, level):
        self.config = self.callback(config, level)
        self.copy_to_parameter_server()
        return self.config
   
    def calc_level(self, config1, config2):
        level = 0
        for param in self.type.config_description:
            if config1.__getattribute__(param['name']) != config2.__getattribute__(param['name']):
                level = level | param['level']
        return level

    def clamp(self, config): 
        for param in self.type.config_description: 
            maxval = self.type.max.__getattribute__(param['name']) 
            minval = self.type.min.__getattribute__(param['name']) 
            val = config.__getattribute__(param['name'])  
            if val > maxval and maxval != "": 
                config.__setattr__(param['name'], maxval) 
            elif val < minval and minval != "": 
                config.__setattr__(param['name'], minval) 

    def get_callback(self, req):
        resp = self.type.GetClass._response_class()
        resp.max = self.type.max
        resp.min = self.type.min
        resp.defaults = self.type.defaults
        resp.config = self.config
        return resp

    def set_callback(self, req):
        self.clamp(req.config)
        return self.change_config(req.config, self.calc_level(req.config, self.config))


