#include "climb_util/parameterized.hpp"

void Parameterized::defaultParameters()
{
  for (const auto & param : getParameters()) {
    setParameter(param.name, param.default_value);
  }
}
