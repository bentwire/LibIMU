#ifndef __VECTOR3D_H__
#define __VECTOR3D_H__

namespace IMU
{

/** @class Vector3D
	*
	* This class implements the Vector3D type and all standard methods.
	*
	*/
template <typename T>
class Vector3D
{
				public:

				/** Default constructor.
				 */
				Vector3D(void) : x(0), y(0), z(0) {};

				/** 
					* Construct from base type triple
					*
					* @param x value to set the X axis value to.
					* @param y value to set the Y axis value to.
					* @param z value to set the Z axis value to.
					*/
				Vector3D(T x, T y, T z) : x(x), y(y), z(z) {};

				/** 
					* Construct from base type array.
					*
					* @param v pointer to array of values (x, y, z) of type T
					*/
				Vector3D(T *v) __attribute__((__nonnull__)): x(v[0]), y(v[1]), z(v[2]) {};

				/** Copy constructors
					*/
				Vector3D(const volatile Vector3D<T> & in) : x(in.x), y(in.y), z(in.z) {};
				Vector3D(const Vector3D<T> & in) : x(in.x), y(in.y), z(in.z) {};

				/** 
					* Set the X, Y and Z components of the vector.
					*
					* @param x value to set the X axis value to.
					* @param y value to set the Y axis value to.
					* @param z value to set the Z axis value to.
					*
					* @return The modified Vector3D<T> object.
					*/
				Vector3D<T>           & set(const T x, const T y, const T z)          { this->x = x, this->y = y, this->z = z; return *this; };
				Vector3D<T>  volatile & set(const T x, const T y, const T z) volatile { this->x = x, this->y = y, this->z = z; return *this; };

				T												X(void) const { return x; };
				T												Y(void) const { return y; };
				T												Z(void) const { return z; };

				/** 3D dot product
					*/
				T												dot(const Vector3D<T> & v) const { return x*v.x+y*v.y+z*v.z; };

				/** 3D cross product
					*/
				Vector3D<T>		cross(const Vector3D<T> & v) const { return Vector3D<T>((y*v.z-z*v.y), (z*v.x-x*v.z), (x*v.y-y*v.x)); };

				/** Vector length
					*/
				T												length(void) const { return sqrt(dot(*this)); };

				/** Vector norm (returns normalized copy).
					*
					* This function returns a normalized copy of the vector.
					*
					* @return The normalized copy of the vector.
					*/
				Vector3D<T>		norm(void) const { T invlen = 1 / this->length(); return Vector3D<T>( x / invlen, y / invlen, z / invlen ); };

				/** Vector norm (in place).
					*
					* This method normalizes the vector (in place) and returns a reference to the vector.
					*
					* @return The normalized vector.
					*/
				Vector3D<T>		& normalize(void)  { T invlen = 1 / this->length(); x = x * invlen, y = y * invlen, z = z * invlen; return *this; };

				// Operators
				//Vector3D<T> & operator -  (const Vector3D<T> & rhs) const { return Vector3D<T>(-x, -y, -z); };  
				Vector3D<T> & operator += (const Vector3D<T> & rhs) { x += rhs.x, y += rhs.y, z += rhs.z; return *this; };
				Vector3D<T> & operator -= (const Vector3D<T> & rhs) { x -= rhs.x, y -= rhs.y, z -= rhs.z; return *this; };
				//Vector3D<T> & operator *= (const Vector3D<T> & rhs);
				//Vector3D<T> & operator /= (const Vector3D<T> & rhs);

				Vector3D<T> & operator += (const T & rhs) { x += rhs, y += rhs, z += rhs; return *this; };
				Vector3D<T> & operator -= (const T & rhs) { x -= rhs, y -= rhs, z -= rhs; return *this; };
				Vector3D<T> & operator *= (const T & rhs) { x *= rhs, y *= rhs, z *= rhs; return *this; };
				Vector3D<T> & operator /= (const T & rhs) { x /= rhs, y /= rhs, z /= rhs; return *this; };

//				Vector3D<T>	volatile & operator  = (const T & rhs) volatile { x = rhs.x, y = rhs.y, z = rhs.z; return *this; };
//				Vector3D<T>          & operator  = (const T & rhs)          { x = rhs.x, y = rhs.y, z = rhs.z; return *this; };
				Vector3D<T>	         &	operator  = (const Vector3D<T> & rhs)            { x = rhs.x, y = rhs.y, z = rhs.z; return *this; };
				Vector3D<T>	volatile &	operator  = (const Vector3D<T> & rhs)   volatile { x = rhs.x, y = rhs.y, z = rhs.z; return *this; };
				//Vector3D<T>	         &	operator  = (const Quaternion<T> & rhs)          { x = rhs.x, y = rhs.y, z = rhs.z; return *this; };
				//Vector3D<T>	volatile &	operator  = (const Quaternion<T> & rhs) volatile { x = rhs.x, y = rhs.y, z = rhs.z; return *this; };

				protected:

				T x, y, z;
}; // class Vector3D

template <typename T>
				inline Vector3D<T> operator + (const Vector3D<T> &lhs, const Vector3D<T> &rhs) { Vector3D<T> res = lhs; res += rhs; return res; };
template <typename T>
				inline Vector3D<T> operator - (const Vector3D<T> &lhs, const Vector3D<T> &rhs) { Vector3D<T> res = lhs; res -= rhs; return res; };
//template <typename T>
//				inline volatile Vector3D<T> operator + (volatile Vector3D<T> &lhs, volatile Vector3D<T> &rhs) { Vector3D<T> res = lhs; res += rhs; return res; };
//template <typename T>
//				inline volatile Vector3D<T> operator - (volatile Vector3D<T> &lhs, volatile Vector3D<T> &rhs) { Vector3D<T> res = lhs; res -= rhs; return res; };
//template <typename T>
//				inline Vector3D<T> volatile operator - (const Vector3D<T> &lhs, const Vector3D<T> &rhs) volatile { volatile Vector3D<T> res = lhs; res -= rhs; return res; };
template <typename T>
				inline Vector3D<T> operator * (const Vector3D<T> &lhs, const Vector3D<T> &rhs) { Vector3D<T> res = lhs; res *= rhs; return res; };

template <typename T>
				inline Vector3D<T> operator + (const Vector3D<T> &lhs, const T &rhs) { Vector3D<T> res = lhs; res += rhs; return res; };
//template <typename T>
//				inline volatile Vector3D<T> & operator + (const volatile Vector3D<T> &lhs, const T &rhs) { Vector3D<T> res = lhs; res += rhs; return res; };
template <typename T>
				inline Vector3D<T> operator - (const Vector3D<T> &lhs, const T &rhs) { Vector3D<T> res = lhs; res -= rhs; return res; };
template <typename T>
				inline Vector3D<T> operator * (const Vector3D<T> &lhs, const T &rhs) { Vector3D<T> res = lhs; res *= rhs; return res; };
template <typename T>
				inline Vector3D<T> operator / (const Vector3D<T> &lhs, const T &rhs) { Vector3D<T> res = lhs; res /= rhs; return res; };
//template <typename T>
//				inline volatile Vector3D<T>  & operator * (const volatile Vector3D<T> &lhs, const T &rhs) { Vector3D<T> res = lhs; res *= rhs; return res; };
template <typename T>
				inline Vector3D<T> operator * (const T &lhs, const Vector3D<T> &rhs) { Vector3D<T> res = rhs; res *= lhs; return res; };



}; // namespace IMU

#endif //__VECTOR3D_H__
