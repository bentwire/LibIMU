#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <math.h>

#include <Vector3D.h>

namespace IMU 
{

template<typename T>
class Quaternion : private Vector3D<T>
{
    public:
    
    /** Default constructor.
     */
    Quaternion(void) : Vector3D<T>(0, 0, 0), w(1) {};

    /** 
     * Construct from base type quad
     *
     * @param w value to set the W axis value to.
     * @param x value to set the X axis value to.
     * @param y value to set the Y axis value to.
     * @param z value to set the Z axis value to.
     */
    Quaternion(const T w, const T x, const T y, const T z) : Vector3D<T>(x, y, z), w(w) {};

    /**
     * Construct pure real Quaternion
     *
     * The vector (imaginary) part of the quaternion is set to 0, so this is a real Quaternion
     *
     * @param real Value of the real part of the Quaternion.
     */
    Quaternion(const T & real) : Vector3D<T>(0, 0, 0), w(real) {};

    /**
     * Construct from Vector3D<T>
     *
     * The real part of the quaternion is set to 0, so this is a pure imaginary Quaternion
     *
     * @param vec Vector3D<T> to use as vector part of Quaternion.
     */
    Quaternion(const Vector3D<T> & vec) : Vector3D<T>(vec), w(0) {};

    /** 
     * Construct from base type array.
     *
     * @param v pointer to array of values [w, x, y, z] of type T
     */
    Quaternion(const T *q) __attribute__((__nonnull__)) : w(q[0]), Vector3D<T>(q[1], q[2], q[3]) {};

    /**
     * Construct from Angle/Axis pair.
     *
     * This can be used to construct a Quaternion that defines rotation about an axis.
     *
     * @param theta The angle part.
     * @param axis  The axis part.  Must be normalized Vector3D<T>
     *
     * This should probably be an external friend function, not a constructor...
     */
    Quaternion(const T & theta, const Vector3D<T> axis);

    /**
     * Copy constructors.
     */
    Quaternion(const Quaternion<T> & rhs) : Vector3D<T>(rhs), w(rhs.w) {};

    /** 
     * Set the W, X, Y and Z components of the Quaternion.
     *
     * @param w value to set the W axis value to.
     * @param x value to set the X axis value to.
     * @param y value to set the Y axis value to.
     * @param z value to set the Z axis value to.
     *
     * @return The modified Quaternion<T> object.
     */
    Quaternion<T>  & set(const T w, const T x, const T y, const T z) { this->w = w, this->x = x, this->y = y, this->z = z; return *this; };
    Quaternion<T>  volatile & set(const T w, const T x, const T y, const T z) volatile { this->w = w, this->x = x, this->y = y, this->z = z; return *this; };

    // Attribute readers...
    inline T     W(void) const { return this->w; };
    inline T     X(void) const { return this->x; };
    inline T     Y(void) const { return this->y; };
    inline T     Z(void) const { return this->z; };
    inline T     real(void) const { return this->w; };
    Vector3D<T>  imag(void) const { return *this; };

    // Quaternion Operations
    T                   dot(const Quaternion<T> & v) const  { return w*v.w+this->Vector3D<T>::dot(v); };
    T                   length(void)                 const  { return sqrt(dot(*this)); };
    Quaternion<T>       conj(void)                   const  { return Quaternion<T>(this->w, -this->x, -this->y, -this->z); };
    Quaternion<T>   &   normalize(void);
    Quaternion<T>       norm(void)                   const;

    
    // Methods that return or operate on a Vector3D<T>
    Vector3D<T>         rot(const Vector3D<T> & vec)             const;
    Vector3D<T>         getEulerAngles(void) const;
    Vector3D<T>         gVec(void) const;

    //Vector3D<T>      gVec(void) volatile { return Vector3D<T>( 2 * (this->x * this->z - this->w * this->y),
    //                                                           2 * (this->w * this->x + this->y * this->z),
    //                                                           this->w * this->w - this->x * this->x - this->y * this->y + this->z * this->z );
    //                                                                                                                                };

    
    // Operators
    Quaternion<T> & operator += (const T & rhs) { w += rhs, this->Vector3D<T>::operator+=(rhs); return *this; };
    Quaternion<T> & operator -= (const T & rhs) { w -= rhs, this->Vector3D<T>::operator-=(rhs); return *this; };
    Quaternion<T> & operator *= (const T & rhs) { w *= rhs, this->Vector3D<T>::operator*=(rhs); return *this; };
    Quaternion<T> & operator /= (const T & rhs) { w /= rhs, this->Vector3D<T>::operator/=(rhs); return *this; };

    Quaternion<T> & operator += (const Quaternion<T> & rhs) { w += rhs.w, this->Vector3D<T>::operator+=(rhs); return *this; };
    Quaternion<T> & operator -= (const Quaternion<T> & rhs) { w -= rhs.w, this->Vector3D<T>::operator-=(rhs); return *this; };
    Quaternion<T> & operator *= (const Quaternion<T> & rhs);
    //Quaternion<T> & operator /= (const Quaternion<T> & rhs);

    Quaternion<T>   operator *  (const Quaternion<T> & rhs) const;
    Quaternion<T>   operator /  (const Quaternion<T> & rhs) const;

    Quaternion<T>            &  operator  = (const Quaternion<T> & rhs)          { Vector3D<T>::x = rhs.Vector3D<T>::x, Vector3D<T>::y = rhs.Vector3D<T>::y, Vector3D<T>::z = rhs.Vector3D<T>::z, w = rhs.w; return *this; };
    Quaternion<T>   volatile &  operator  = (const Quaternion<T> & rhs) volatile { Vector3D<T>::x = rhs.Vector3D<T>::x, Vector3D<T>::y = rhs.Vector3D<T>::y, Vector3D<T>::z = rhs.Vector3D<T>::z, w = rhs.w; return *this; };

    protected:

    T w;

    static const T MIN_NORM = 1.0e-7;

}; // class Quaternion


template <typename T>
Quaternion<T> 
Quaternion<T>::norm(void) const
{
    T len = this->length();
    if (len > MIN_NORM)
    {
        return Quaternion<T>(this->w / len, this->x / len, this->y / len, this->z / len);
    }
    return *this;   
}

template <typename T>
Quaternion<T> & 
Quaternion<T>::normalize(void)
{
    T len = this->length();
    if (len > MIN_NORM)
    {
        this->w /= len;
        this->x /= len;
        this->y /= len;
        this->z /= len;
    }
    return *this;   
}

template <typename T>
Vector3D<T>
Quaternion<T>::rot(const Vector3D<T> & vec) const
{
    Quaternion p(vec);
    p = *this * p;
    p = p * this->conj();

    return Vector3D<T>(p.x, p.y, p.z);
}

template <typename T>
Vector3D<T>
Quaternion<T>::getEulerAngles(void) const
{
    T psi   = atan2(2.0f * this->x * this->y - 2.0f * this->w * this->z, 2.0f * this->w * this->w + 2.0f * this->x * this->x - 1);
    T theta = -asin(2.0f * this->x * this->z + 2.0f * this->w * this->y);
    T phi   = atan2(2.0f * this->y * this->z - 2.0f * this->w * this->x, 2.0f * this->w * this->w + 2.0f * this->z * this->z - 1);

    return Vector3D<T>(phi, theta, psi);
}

template <typename T>
Vector3D<T>
Quaternion<T>::gVec(void) const 
{ 
    return Vector3D<T>( 2 * (this->x * this->z - this->w * this->y),
                        2 * (this->w * this->x + this->y * this->z),
                        this->w * this->w - this->x * this->x - this->y * this->y + this->z * this->z );
}


template <typename T>
Quaternion<T> 
Quaternion<T>::operator * (const Quaternion<T> & rhs) const
{
    T w = this->w * rhs.w - this->Vector3D<T>::dot(rhs);
    T x = this->x * rhs.w + this->w * rhs.x + this->y * rhs.z - this->z * rhs.y;
    T y = this->y * rhs.w + this->w * rhs.y + this->z * rhs.x - this->x * rhs.z;
    T z = this->z * rhs.w + this->w * rhs.z + this->x * rhs.y - this->y * rhs.x;

    return Quaternion<T>(w, x, y, z);
}

template <typename T>
Quaternion<T> & 
Quaternion<T>::operator *= (const Quaternion<T> & rhs)
{
    T w = this->w * rhs.w - this->Vector3D<T>::dot(rhs);
    T x = this->x * rhs.w + this->w * rhs.x + this->y * rhs.z - this->z * rhs.y;
    T y = this->y * rhs.w + this->w * rhs.y + this->z * rhs.x - this->x * rhs.z;
    T z = this->z * rhs.w + this->w * rhs.z + this->x * rhs.y - this->y * rhs.x;

    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;

    return *this;
}

template <typename T>
inline Quaternion<T> 
operator + (const Quaternion<T> &lhs, const Quaternion<T> &rhs) { Quaternion<T> res = lhs; res += rhs; return res; };

template <typename T>
inline Quaternion<T> 
operator - (const Quaternion<T> &lhs, const Quaternion<T> &rhs) { Quaternion<T> res = lhs; res -= rhs; return res; };

template <typename T>
inline Quaternion<T> 
operator + (const Quaternion<T> &lhs, const T &rhs) { Quaternion<T> res = lhs; res += rhs; return res; };

template <typename T>
inline Quaternion<T> 
operator - (const Quaternion<T> &lhs, const T &rhs) { Quaternion<T> res = lhs; res -= rhs; return res; };

}; // namespace IMU

#endif //__QUATERNION_H__
