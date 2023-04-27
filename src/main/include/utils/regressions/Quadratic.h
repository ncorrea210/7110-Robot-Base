#pragma once

namespace hb {
  
    /**
   * The values entered in the constructor shoudld be the same ones coming from this desmos:
   * https://www.desmos.com/calculator/jjwe4f3fl0
   * 
   * The numbers should be entered into the contructor in the order found in desmos but for reference they will be this:
   * ax^2 + bx + c
   * 
   */
  class Quadratic { 

    public:

      explicit Quadratic(double a, double b, double c);

      double Calculate(double x) const;

    private:

      double m_a, m_b, m_c;

  };
}