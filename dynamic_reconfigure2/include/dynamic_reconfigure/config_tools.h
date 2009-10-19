#ifndef __DYNAMIC_RECONFIGURE__CONFIG_TOOLS__
#define __DYNAMIC_RECONFIGURE__CONFIG_TOOLS__

#include <string>

namespace dynamic_reconfigure
{

class ConfigTools
{
  static void std::vector<bool> &getVectorForType(ParameterSet &set, bool val)
  {
    return set.bools;
  }
  
  static void std::vector<int> &getVectorForType(ParameterSet &set, int val)
  {
    return set.ints;
  }
  
  static void std::vector<std::string> &getVectorForType(ParameterSet &set, std::string &val)
  {
    return set.strs;
  }
  
  static void std::vector<double> &getVectorForType(ParameterSet &set, double val)
  {
    return set.doubles;
  }
  
  static void BoolParameter makeKeyValuePair(std::string &name, bool val)
  {
    return BoolParameter(name, val);
  }

  static void IntParameter makeKeyValuePair(std::string &name, int val)
  {
    return IntParameter(name, val);
  }

  static void StrParameter makeKeyValuePair(std::string &name, const std::string &)
  {
    return StrParameter(name, val);
  }

  static void DoubleParameter makeKeyValuePair(std::string &name, double val)
  {
    return DoubleParameter(name, val);
  }

  template <class T>
  static void appendParameter(ParameterSet &set, const std::string &name, T &val)
  {
    getVectorForType(set, val).push_back(makeKeyValuePair(name, val))
  }

  template <class V, T>
  static bool getParameter(V &vec, const std::string &name, T &val)
  {
    for (V.const_iterator i = vec.begin(); i != v.end(); i++)
      if (i->name == name)
      {
        val = i->value;
        return true;
      }
    return false;
  }

  template <class T>
  static bool getParameter(ParameterSet &set, const std::string &name, T &val)
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
  static void appendNames(ParameterSet &set, std::vector<std::string> &names)*/
};

}

#endif
