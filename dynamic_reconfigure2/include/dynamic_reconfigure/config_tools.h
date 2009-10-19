#ifndef __DYNAMIC_RECONFIGURE__CONFIG_TOOLS__
#define __DYNAMIC_RECONFIGURE__CONFIG_TOOLS__

#include <string>
#include <vector>
#include <dynamic_reconfigure2/Config.h>

namespace dynamic_reconfigure
{

class ConfigTools
{
  static std::vector<bool> &getVectorForType(dynamic_reconfigure2::Config &set, bool val)
  {
    return set.bools;
  }
  
  static std::vector<int> &getVectorForType(dynamic_reconfigure2::Config &set, int val)
  {
    return set.ints;
  }
  
  static std::vector<std::string> &getVectorForType(dynamic_reconfigure2::Config &set, std::string &val)
  {
    return set.strs;
  }
  
  static std::vector<double> &getVectorForType(dynamic_reconfigure2::Config &set, double val)
  {
    return set.doubles;
  }
  
  static dynamic_reconfigure2::BoolParameter makeKeyValuePair(std::string &name, bool val)
  {
    
    dynamic_reconfigure2::BoolParameter param = { name, val };
    return param;
  }

  static dynamic_reconfigure2::IntParameter makeKeyValuePair(std::string &name, int val)
  {
    return dynamic_reconfigure2::IntParameter(name, val);
  }

  static dynamic_reconfigure2::StrParameter makeKeyValuePair(std::string &name, const std::string &val)
  {
    return dynamic_reconfigure2::StrParameter(name, val);
  }

  static dynamic_reconfigure2::DoubleParameter makeKeyValuePair(std::string &name, double val)
  {
    return dynamic_reconfigure2::DoubleParameter(name, val);
  }

  template <class T>
  static void appendParameter(dynamic_reconfigure2::Config &set, const std::string &name, T &val)
  {
    getVectorForType(set, val).push_back(makeKeyValuePair(name, val));
  }

  template <class V, class T>
  static bool getParameter(V &vec, const std::string &name, T &val)
  {
    for (V::const_iterator i = vec.begin(); i != vec.end(); i++)
      if (i->name == name)
      {
        val = i->value;
        return true;
      }
    return false;
  }

  template <class T>
  static bool getParameter(dynamic_reconfigure2::Config &set, const std::string &name, T &val)
  {
    return getParameter(getVectorForType(set, val), val);
  }

  /*template <class V> 
  static void appendNames(V &vec, std::vector<std::string> &names)
  {
    for (V.const_iterator i = vec.begin(); i != v.end(); i++)
      names.push_back(i->name);
  }

  template <>
  static void appendNames(dynamic_reconfigure2::Config &set, std::vector<std::string> &names)*/
};

}

#endif
