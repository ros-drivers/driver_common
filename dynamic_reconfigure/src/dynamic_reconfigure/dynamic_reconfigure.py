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

class DynamicReconfigure:
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
        req = self.set_service_class._request_class(config)
        #print self.set_service(req).config
        return req

