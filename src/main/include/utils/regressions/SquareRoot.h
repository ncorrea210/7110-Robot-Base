#pragma once 

namespace hb {
  
  /**
   * The values entered in the constructor shoudld be the same ones coming from this desmos:
   * https://www.desmos.com/calculator/jjwe4f3fl0
   * 
   * The numbers should be entered into the contructor in the order found in desmos but for reference they will be this:
   * a * sqrt(b * x - h) + k
   * 
   */
  class SquareRoot {

    public:

      SquareRoot(double a, double b, double h, double k);

      double Calculate(double x) const;

    private:

      double m_a, m_b, m_h, m_k;
  };
}