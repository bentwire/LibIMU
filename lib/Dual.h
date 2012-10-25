#ifndef __DUAL_MATH_H__
#define __DUAL_MATH_H__

#include <iostream>
#include <math.h>

namespace IMU
{

template <typename T>
class Dual
{
				public:
				// Constructors
				Dual(void)              : real(0.0f), epsilon(0.0f)    {};
				Dual(T real)            : real(real), epsilon(0.0f)    {};
				Dual(T real, T epsilon) : real(real), epsilon(epsilon) {};

				Dual<T>   conj(void) const { Dual<T> tmp(real, -epsilon); return tmp; };
				Dual<T>	  inv(void)  const;
				T      	  norm(void)  const;

				// Arithmetic Operators
				Dual<T> & operator +(void) { return *this; };
				Dual<T>   operator -(void);

				Dual<T>   operator *(const Dual<T> & rhs);

				Dual<T> & operator +=(const Dual<T> & rhs);
				Dual<T> & operator -=(const Dual<T> & rhs);
				Dual<T> & operator *=(const Dual<T> & rhs);
				Dual<T> & operator /=(const Dual<T> & rhs);

				Dual<T> & operator +=(const T & rhs);
				Dual<T> & operator -=(const T & rhs);
				Dual<T> & operator *=(const T & rhs);
				Dual<T> & operator /=(const T & rhs);

				// math.h stuff  These should probably all be external "friend" functions..
				Dual<T>   exp(void) const { Dual<T> tmp(exp(real), exp(real) * epsilon); return tmp; };
				Dual<T>   log(void) const { Dual<T> tmp(log(real), epsilon/real); return tmp; };
				Dual<T>   log10(void) const { Dual<T> tmp = this->log() / ::log(10); return tmp; };
				Dual<T>	  pow(const T & rhs) const;
				Dual<T>	  pow(const Dual<T> & rhs) const;
				Dual<T>	  sqrt(void) const;
				T         abs(void) const { return norm(); };
//				T         real(void) const; // Need to fix private var names before I can do this...
				T         imag(void) const { return epsilon; };

				Dual<T>   sin(void) const;
				Dual<T>	  cos(void) const;
				Dual<T>   tan(void) const;


				//friend Dual<T> operator -(const T & lhs, const Dual<T> & rhs);
				template <class D> friend std::ostream & operator <<(std::ostream & out, const Dual<D> & rhs);



				private:
				// The real and epsilon parts of a dual.
				T real;
				T epsilon;

}; // Class Dual

template <typename T>
Dual<T>
Dual<T>::inv(void) const
{ 
				Dual<T> tmp; 
				
				tmp.real    =  real    / (real * real);
				tmp.epsilon = -epsilon / (real * real);
				
				return tmp;
};

template <typename T>
Dual<T>
Dual<T>::operator -(void)
{
				Dual<T> tmp(-real, -epsilon);

				return tmp;
}

template <typename T>
Dual<T>
Dual<T>::operator *(const Dual<T> & rhs)
{
				Dual<T> tmp;
				
				tmp.real    = real * rhs.real;
				tmp.epsilon = real * rhs.epsilon + epsilon * rhs.real;

				return tmp;
}

template <typename T>
Dual<T> & 
Dual<T>::operator +=(const Dual<T> & rhs) 
{ 
				real    += rhs.real;
				epsilon += rhs.epsilon; 
				
				return *this; 
}

template <typename T>
Dual<T> & 
Dual<T>::operator -=(const Dual<T> & rhs) 
{ 
				real    -= rhs.real;
				epsilon -= rhs.epsilon; 
				
				return *this;
}

template <typename T>
Dual<T> & 
Dual<T>::operator *=(const Dual<T> & rhs) 
{ 
				real    *= rhs.real;
				epsilon  = real * rhs.epsilon + epsilon * rhs.real; 
				
				return *this;
}

template <typename T>
Dual<T> & 
Dual<T>::operator /=(const Dual<T> & rhs) 
{
				real     = real / rhs.real;
				epsilon  = ( epsilon * rhs.real - real * rhs.epsilon ) / (rhs.real * rhs.real);

				return *this;
}

template <typename T>
Dual<T> & 
Dual<T>::operator +=(const T & rhs) 
{ 
				real    += rhs.real;
				
				return *this; 
}

template <typename T>
Dual<T> & 
Dual<T>::operator -=(const T & rhs) 
{ 
				real    -= rhs.real;
				
				return *this;
}

template <typename T>
Dual<T> & 
Dual<T>::operator *=(const T & rhs) 
{ 
				real    *= rhs;
				epsilon *= rhs; 
				
				return *this;
}

template <typename T>
Dual<T>
Dual<T>::pow(const T & rhs) const
{
				static const T min_real = 1e-15;
				T real_checked;
				T epsilon_mul;
				Dual<T> tmp;

				real_checked = real;
#if 1
				if (fabs(real_checked) < min_real)
				{
								if (real_checked >= 0) real_checked =  min_real;
								if (real_checked <  0) real_checked = -min_real;
				}
#endif
				epsilon_mul = rhs * powl(real_checked, (rhs - 1));

				tmp.real    = powl(real, rhs);
				tmp.epsilon = epsilon * epsilon_mul;

				return tmp;
}

template <typename T>
Dual<T>
Dual<T>::pow(const Dual<T> & rhs) const
{
				static const T min_real = 1e-15;
				T real_checked;
				T epsilon_mul;
				T epsilon_add;
				Dual<T> tmp;

				real_checked = real;
#if 1
				if (fabs(real_checked) < min_real)
				{
								if (real_checked >= 0) real_checked =  min_real;
								if (real_checked <  0) real_checked = -min_real;
				}
#endif
				epsilon_mul = rhs.real * powl(real_checked, (rhs.real - 1));
				epsilon_add = rhs.epsilon * ::powl(real_checked, rhs.real) * ::logl(real);

				tmp.real    = powl(real, rhs.real);
				tmp.epsilon = epsilon * epsilon_mul + epsilon_add;

				return tmp;
}

template <typename T>
Dual<T>
Dual<T>::sqrt(void) const
{
				Dual<T> tmp;

				tmp.real = ::sqrtl(real);
				tmp.epsilon = epsilon / (2 * ::sqrtl(real));

				return tmp;
}

template <typename T>
T
Dual<T>::norm(void) const
{
				return ::sqrtl(real*real + epsilon*epsilon);
}

template <typename T> 
std::ostream & operator <<(std::ostream & out, const Dual<T> & rhs)
{
				out << "(" << rhs.real << "," << rhs.epsilon << ")";

				return out;
}

}; // Namespace IMU

#endif

