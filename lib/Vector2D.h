#ifndef __VECTOR2D_H__
#define __VECTOR2D_H__

namespace IMU
{

template <typename T>
class Vector2D
{
				public:

				/** Default constructor.
				 */
				Vector2D(void) : x(0), y(0) {};

				/** Construct from base type
					*/
				Vector2D(T x, T y) : x(x), y(y) {};

				/** Construct from base type array
					*/
				Vector2D(T *v) __attribute__((__nonnull__)): x(v[0]), y(v[1]) {};

				/** Copy constructor
					*/
				Vector2D(const Vector2D<T> & rhs) : x(rhs.x), y(rhs.y) {};

				// Attribute readers...
				T X(void) const { return x; };
				T Y(void) const { return y; };

				T dot(const Vector2D<T> & v) const { return x*v.x+y*v.y; };
				T length(void) const { return sqrt(dot(*this)); };

				// Operators
				Vector2D<T> & operator += (const Vector2D<T> & rhs);
				Vector2D<T> & operator -= (const Vector2D<T> & rhs);
				Vector2D<T> & operator *= (const Vector2D<T> & rhs);
				Vector2D<T> & operator /= (const Vector2D<T> & rhs);

				Vector2D<T> & operator += (const T & rhs) { x += rhs, y += rhs; return *this; };
				Vector2D<T> & operator -= (const T & rhs) { x -= rhs, y -= rhs; return *this; };
				Vector2D<T> & operator *= (const T & rhs) { x *= rhs, y *= rhs; return *this; };
				Vector2D<T> & operator /= (const T & rhs) { x /= rhs, y /= rhs; return *this; };

				private:

				T x, y;
}; // class Vector2D

template <typename T>
inline Vector2D<T> 
operator + (const Vector2D<T> &lhs, const Vector2D<T> &rhs) 
{ 
				Vector2D<T> res = lhs;
				
				res += rhs; 
				return res; 
}

template <typename T>
inline Vector2D<T> 
operator - (const Vector2D<T> &lhs, const Vector2D<T> &rhs) 
{
				Vector2D<T> res = lhs;
				
				res -= rhs; 
				return res; 
}

template <typename T>
inline Vector2D<T> 
operator + (const Vector2D<T> &lhs, const T &rhs) 
{
				Vector2D<T> res = lhs;
				
				res += rhs; 
				return res; 
}

template <typename T>
inline Vector2D<T>
operator - (const Vector2D<T> &lhs, const T &rhs) 
{ 
				Vector2D<T> res = lhs;
				
				res -= rhs;
				return res;
}

}; // namespace IMU
#endif //__VECTOR2D_H__
