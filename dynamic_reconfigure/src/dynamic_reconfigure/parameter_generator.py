#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2008, Willow Garage, Inc.
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

# Author: Blaise Gassend

# Given a set of parameters, generates the messages, service types, and
# classes to allow runtime reconfiguration. Documentation of a node's
# parameters is a handy byproduct.

## @todo
# Need to check types of min max and default
# Need to put sane error on exceptions

import roslib; roslib.load_manifest("dynamic_reconfigure")
import roslib.packages
from string import Template
import os
import inspect
import string 

# Convenience names for types
str_t = "str"
bool_t = "bool"
int_t = "int"
double_t = "double"

class ParameterGenerator:
    minval = {
            'int' : 'INT_MIN',
            'double' : '-std::numeric_limits<double>::infinity()',
            'str' : '',
            'bool' : False,
            }
            
    maxval = {
            'int' : 'INT_MAX',
            'double' : 'std::numeric_limits<double>::infinity()',
            'str' : '',
            'bool' : True,
            }
    
    defval = {
            'int' : 0,
            'double' : 0,
            'str' : '',
            'bool' : False,
            }
            
    def __init__(self, pkgname, nodename, name):
        self.parameters = []
        self.pkgname = pkgname
        self.pkgpath = roslib.packages.get_pkg_dir(pkgname)
        self.dynconfpath = roslib.packages.get_pkg_dir("dynamic_reconfigure")
        self.name = name
        self.nodename = nodename
        self.msgname = name+"Config"

    def add(self, name, type, level, description, default = None, min = None, max = None):
        if min == None:
            min = self.minval[type]
        if max == None:
            max = self.maxval[type]
        if default == None:
            default = self.defval[type]
        self.parameters.append({
            'name' : name,
            'type' : type,
            'default' : default,
            'level' : level,
            'description' : description,
            'min' : min,
            'max' : max,
            'srcline' : inspect.currentframe().f_back.f_lineno,
            'srcfile' : inspect.getsourcefile(inspect.currentframe().f_back.f_code),
            })

    def mkdirabs(self, path, second_attempt = False):
        if os.path.isdir(path):
            pass
        elif os.path.isfile(path):
            raise OSError("Error creating directory %s, a file with the same name exists" %path)
        elif second_attempt: # An exception occurred, but we still don't know why.
            raise
        else:
            head, tail = os.path.split(path)
            if head and not os.path.isdir(head):
                self.mkdir(head)
            if tail:
                try:
                    os.mkdir(path)
                except OSError:
                    # Probably got created by somebody else, lets check.
                    self.mkdirabs(path, True)

    def mkdir(self, path):
        path = os.path.join(self.pkgpath, path)
        self.mkdirabs(path)

    def generate(self):
        #print '**************************************************************'
        #print '**************************************************************'
        print Template("Generating reconfiguration files for $name in $pkgname").\
                substitute(name=self.name, pkgname = self.pkgname)
        #print '**************************************************************'
        #print '**************************************************************'
        self.generateconfigmanipulator()
        self.generatemsg()
        self.generatesetsrv()
        self.generategetsrv()
        self.generatedoc()
        self.generateusage()

    def generatedoc(self):
        self.mkdir("dox")
        f = open(os.path.join(self.pkgpath, "dox", self.msgname+".dox"), 'w')
        #print >> f, "/**"
        print >> f, "\\subsubsection parameters ROS parameters"
        print >> f
        print >> f, "Reads and maintains the following parameters on the ROS server"
        print >> f
        for param in self.parameters:
            print >> f, Template("- \\b \"~$name\" : \\b [$type] $description min: $min, default: $default, max: $max").substitute(param)
        print >> f
        #print >> f, "*/"
        f.close()

    def generateusage(self):
        self.mkdir("dox")
        f = open(os.path.join(self.pkgpath, "dox", self.msgname+"-usage.dox"), 'w')
        #print >> f, "/**"
        print >> f, "\\subsubsection usage Usage"
        print >> f, '\\verbatim'
        print >> f, Template('<node name="$nodename" pkg="$pkgname" type="$nodename">').\
                substitute(pkgname = self.pkgname, nodename = self.nodename)
        for param in self.parameters:
            print >> f, Template('  <param name="$name" type="$type" value="$default" />').substitute(param)
        print >> f, '</node>'
        print >> f, '\\endverbatim'
        print >> f
        #print >> f, "*/"
        f.close()

    def crepr(self, param, val):
        type = param["type"]
        if type == 'str':
            return '"'+val+'"'
        if type in [ 'int', 'double']:
            return str(val)
        if  type == 'bool':
            return { True : 1, False : 0 }[val]
        raise TypeError(type)
#        if type == 'string':
#            return '"'+val+'"'
#        if 'uint' in type:
#            return str(val)+'ULL'
#        if 'int' in type:
#            return str(val)+'LL'
#        if 'time' in type:
#            return 'ros::Time('+str(val)+')'
#        if 'duration' in type:
#            return 'ros::Duration('+str(val)+')'
#        if  'float' in types:
#            return str(val)

    def appendline(self, list, text, param, value = None):
        if value == None:
            val = ""
        else:
            val = self.crepr(param, param[value])
        list.append(Template('      '+text).substitute(param, v=val))
        #list.append(Template('#line $srcline "$srcfile"\n      '+text).substitute(param, v=val))
    
    def generateconfigmanipulator(self):
        # Read the configuration manipulator template and insert line numbers and file name into template.
        templatefile = os.path.join(self.dynconfpath, "templates", "ConfigReconfigurator.h")
        templatelines = []
        templatefilesafe = templatefile.replace('aa', 'bb') # line directive does backslash expansion.
        curline = 1
        f = open(templatefile)
        for line in f:
            curline = curline + 1
            templatelines.append(Template(line).safe_substitute(linenum=curline,filename=templatefilesafe))
        f.close()
        template = ''.join(templatelines)
        
        # Write the configuration manipulator.
        self.mkdir(os.path.join("cfg", "cpp", self.pkgname))
        f = open(os.path.join(self.pkgpath, "cfg", "cpp", self.pkgname, self.name+"Reconfigurator.h"), 'w')
        readparam = []
        writeparam = []
        changelvl = []
        defminmax = []
        for param in self.parameters:
            self.appendline(defminmax, "min.$name = $v;", param, "min")
            self.appendline(defminmax, "max.$name = $v;", param, "max")
            self.appendline(defminmax, "defaults.$name = $v;", param, "default")
            self.appendline(changelvl, "if (config1.$name != config2.$name) changelvl |= $level;", param)
            # We introduce tmp_name because bool is not supported in a .msg file.
            if param['type'] == 'bool':
                self.appendline(writeparam, "bool tmp_$name = config.$name;", param)
                self.appendline(writeparam, "nh.setParam(\"~$name\", tmp_$name);", param)
                self.appendline(readparam, "bool tmp_$name = config.$name;", param)
                self.appendline(readparam, "nh.getParam(\"~$name\", tmp_$name, true);", param)
                self.appendline(readparam, "config.$name = tmp_$name;", param)
            else:
                self.appendline(writeparam, "nh.setParam(\"~$name\", config.$name);", param)
                self.appendline(readparam, "nh.getParam(\"~$name\", config.$name, true);", param)
        defminmax = string.join(defminmax, '\n')
        changelvl = string.join(changelvl, '\n')
        writeparam = string.join(writeparam, '\n')
        readparam = string.join(readparam, '\n')
        f.write(Template(template).substitute(uname=self.name.upper(), name = self.name, 
            pkgname = self.pkgname, readparam = readparam, writeparam = writeparam, 
            changelvl = changelvl, defminmax = defminmax))
        f.close()

    def msgtype(self, type):
        return { 'int' : 'int32', 'bool' : 'int8', 'str' : 'string', 'double' : 'float64' }[type]

    def generatemsg(self):
        self.mkdir("msg")
        f = open(os.path.join(self.pkgpath, "msg", self.msgname+".msg"), 'w')
        print >> f, "# This is an autogerenated file. Please do not edit."
        print >> f, ""
        for param in self.parameters:
            print >> f, Template("$type $name # $description").substitute(param, type=self.msgtype(param['type']))
        f.close()

    def generategetsrv(self):
        self.mkdir("srv")
        f = open(os.path.join(self.pkgpath, "srv", "Get"+self.msgname+".srv"), 'w')
        print >> f, "# This is an autogerenated file. Please do not edit."
        print >> f, ""
        print >> f, "---" 
        print >> f, self.msgname, "config", "# Current configuration of node."
        print >> f, self.msgname, "defaults", "# Minimum values where appropriate."
        print >> f, self.msgname, "min", "# Minimum values where appropriate."
        print >> f, self.msgname, "max", "# Maximum values where appropriate."
        f.close()

    def generatesetsrv(self):
        self.mkdir("srv")
        f = open(os.path.join(self.pkgpath, "srv", "Set"+self.msgname+".srv"), 'w')
        print >> f, "# This is an autogerenated file. Please do not edit."
        print >> f, self.msgname, "config", "# Requested node configuration."
        print >> f, "---"        
        print >> f, self.msgname, "config", "# What the node's configuration was actually set to."
        f.close()
