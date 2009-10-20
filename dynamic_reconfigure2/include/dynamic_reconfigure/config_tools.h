#ifndef __DYNAMIC_RECONFIGURE__CONFIG_TOOLS__
#define __DYNAMIC_RECONFIGURE__CONFIG_TOOLS__

#include <string>
#include <vector>
#include <dynamic_reconfigure2/Config.h>

namespace dynamic_reconfigure
{

class ConfigTools
{
public:
  static std::vector<dynamic_reconfigure2::BoolParameter> &getVectorForType(dynamic_reconfigure2::Config &set, const bool val)
  {
    return set.bools;
  }
  
  static std::vector<dynamic_reconfigure2::IntParameter> &getVectorForType(dynamic_reconfigure2::Config &set, const int val)
  {
    return set.ints;
  }
  
  static std::vector<dynamic_reconfigure2::StrParameter> &getVectorForType(dynamic_reconfigure2::Config &set, const std::string &val)
  {
    return set.strs;
  }
  
  static std::vector<dynamic_reconfigure2::DoubleParameter> &getVectorForType(dynamic_reconfigure2::Config &set, const double val)
  {
    return set.doubles;
  }
  
  static const std::vector<dynamic_reconfigure2::BoolParameter> &getVectorForType(const dynamic_reconfigure2::Config &set, const bool val)
  {
    return set.bools;
  }
  
  static const std::vector<dynamic_reconfigure2::IntParameter> &getVectorForType(const dynamic_reconfigure2::Config &set, const int val)
  {
    return set.ints;
  }
  
  static const std::vector<dynamic_reconfigure2::StrParameter> &getVectorForType(const dynamic_reconfigure2::Config &set, const std::string &val)
  {
    return set.strs;
  }
  
  static const std::vector<dynamic_reconfigure2::DoubleParameter> &getVectorForType(const dynamic_reconfigure2::Config &set, const double val)
  {
    return set.doubles;
  }
  
  static dynamic_reconfigure2::BoolParameter makeKeyValuePair(const std::string &name, const bool val)
  {
    dynamic_reconfigure2::BoolParameter param;
    param.name = name;
    param.value = val ;
    return param;
  }

  static dynamic_reconfigure2::IntParameter makeKeyValuePair(const std::string &name, const int val)
  {
    dynamic_reconfigure2::IntParameter param;
    param.name = name;
    param.value = val ;
    return param;
  }

  static dynamic_reconfigure2::StrParameter makeKeyValuePair(const std::string &name, const std::string &val)
  {
    dynamic_reconfigure2::StrParameter param;
    param.name = name;
    param.value = val ;
    return param;
  }

  static dynamic_reconfigure2::DoubleParameter makeKeyValuePair(const std::string &name, const double val)
  {
    dynamic_reconfigure2::DoubleParameter param;
    param.name = name;
    param.value = val ;
    return param;
  }

  template <class T>
  static void appendParameter(dynamic_reconfigure2::Config &set, const std::string &name, const T &val)
  {
    getVectorForType(set, val).push_back(makeKeyValuePair(name, val));
  }

  template <class VT, class T>
  static bool getParameter(const std::vector<VT> &vec, const std::string &name, T &val)
  {
    for (typename std::vector<VT>::const_iterator i = vec.begin(); i != vec.end(); i++)
      if (i->name == name)
      {
        val = i->value;
        return true;
      }
    return false;
  }

  template <class T>
  static bool getParameter(const dynamic_reconfigure2::Config &set, const std::string &name, T &val)
  {
    return getParameter(getVectorForType(set, val), name, val);
  }

  static int size(dynamic_reconfigure2::Config &msg)
  {
    return msg.get_bools_size() + msg.get_doubles_size() + msg.get_ints_size() + msg.get_strs_size();
  }

  static void clear(dynamic_reconfigure2::Config &msg)
  {
    msg.bools.clear();
    msg.ints.clear();
    msg.strs.clear();
    msg.doubles.clear();
  }
};

}

#endif
