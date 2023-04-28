#pragma once

namespace hb {
  
  class RegressionBase {

    public:

      virtual double Calculate(double x) const = 0;

    protected: 

      RegressionBase() = default;
  };
}