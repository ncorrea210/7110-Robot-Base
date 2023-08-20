[Back](/docs/utils/Util.md)

### [Util.h](/src/main/include/utils/Util.h)

This file does not contain a class and rather simply contains a defintion and some functions

## Functions and Defintions

### LAMBDA(x)
This is a macro that expands to [this] {return x;} which is useful for the many sitiations where lambdas are necessary. 
Please note that it captures [this] so it is important to be sure that you really want to capture [this].
In most cases this is fine but in some edge cases it is not.

### bool InRange(double val, double target, double epsilon) 
This function checks to see if the read value(val) is within the bounded range around the target given by epsilon 
where the bounded range will be [target - epsilon, target + epsilon]

### int sgn(double x) 
This function can be defined as the derivative of |x| where x=0 is defined at 1. 
This function simply returns the sign of the number passed into it as this is sometimes needed