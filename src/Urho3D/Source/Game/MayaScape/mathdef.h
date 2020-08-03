#pragma once

#include "shared_libs.h"

using namespace Urho3D;

// Deterministic trig functions.
int32_t iSin(uint16_t a);               ///< Returns sin(a*2π >> 16) << 16, rounded to nearest integer. Used as the x component in this game.
int32_t iCos(uint16_t a);               ///< Returns cos(a*2π >> 16) << 16, rounded to nearest integer. Used as the y component in this game.
int32_t iSinR(uint16_t a, int32_t r);   ///< Returns r*sin(a*2π >> 16), with up to 16 bits precision.
int32_t iCosR(uint16_t a, int32_t r);   ///< Returns r*cos(a*2π >> 16), with up to 16 bits precision.
int32_t iSinSR(int32_t a, int32_t s, int32_t r);  ///< Returns r*sin(a*2π/s), with up to 16 bits precision.
int32_t iCosSR(int32_t a, int32_t s, int32_t r);  ///< Returns r*cos(a*2π/s), with up to 16 bits precision.
uint16_t iAtan2(int32_t s, int32_t c);  ///< Returns atan2(s, c)/2π << 16, with a small ±1.5 platform-independent error. Used as atan2(x, y) in this game.
int32_t iSqrt(uint32_t n);              ///< Returns √(n), rounded down.
int32_t i64Sqrt(uint64_t n);            ///< Returns √(n), rounded down.
int32_t iHypot(int32_t x, int32_t y);   ///< Returns √(x² + y²), rounded down. In case of overflow, returns correct result cast to (int32_t).
int32_t iHypot3(int32_t x, int32_t y, int32_t z);  ///< Returns √(x² + y² + z²), rounded down. In case of overflow, returns correct result cast to (int32_t).

/// Returns the given angle, wrapped to the range [-180°; 180°) = [-32768; 32767].
static inline int32_t angleDelta(int32_t a)
{
return (int16_t)a;  // Cast wrapping intended.
}

static inline IntVector3 toVector(Quaternion const &r)
{
    return IntVector3(r.YawAngle(), r.PitchAngle(), r.RollAngle());
}

// vector * vector -> scalar
// Note: glm doesn't provide dot operator for integral vector.
static inline int dot(IntVector2 const &a, IntVector2 const &b)
{
    return a.x_ * b.x_ + a.y_ * b.y_;
}
static inline int dot(IntVector3 const &a, IntVector3 const &b)
{
    return a.x_ * b.x_ + a.y_ * b.y_ + a.z_ * b.z_;
}

// iSinCosR(angle, scalar) -> 2d_vector
static inline IntVector2 iSinCosR(uint16_t a, int32_t r)
{
return IntVector2(iSinR(a, r), iCosR(a, r));
}

// iAtan2(2d_vector) -> angle
static inline int iAtan2(IntVector2 const &a)
{
    return iAtan2(a.x_, a.y_);
}

// iHypot(vector) -> scalar
static inline int iHypot(IntVector2 const &a)
{
    return iHypot(a.x_, a.y_);
}
static inline int iHypot(IntVector3 const &a)
{
    return iHypot3(a.x_, a.y_, a.z_);
}

/*!
 * Rotate v
 * \param v vector to rotate
 * \param angle the amount * 32768/π to rotate in counterclockwise direction
 * \return Result
 */
static inline Vector2 Vector2_Rotate2f(Vector2 v, int angle)
{
Vector2 result;
result.x_ = (v.x_ * iCos((uint16_t)angle) - v.y_ * iSin((uint16_t)angle)) / 65536;
result.y_ = (v.x_ * iSin((uint16_t)angle) + v.y_ * iCos((uint16_t)angle)) / 65536;

return result;
}


/*!
 * Much the same as IntVector2_InCircle except that it works in 3-axis by discarding the z-component and with
 * circles.
 * \param v Vector to test
 * \param c Vector containing the centre of the circle
 * \param r The radius of the circle
 * \return If v falls within the circle
 */
static inline bool IntVector3_InCircle(IntVector3 v, IntVector3 c, unsigned r)
{
//IntVector2 delta = IntVector3(v - c).xy();
    IntVector3 d3 = IntVector3(v - c);
    IntVector2 delta = IntVector2(d3.x_*d3.y_, d3.z_);

// Explictily cast to "unsigned int" because this number never can be
// negative, due to the fact that these numbers are squared. Still GCC
// warns about a comparison of a comparison between an unsigned and a
// signed integer.
return (unsigned int)dot(delta, delta) < r * r;
}


/*!
 * Much the same as IntVector2_InCircle except that it works in 3-axis and with
 * spheres.
 * The equation used is also ever so slightly different:
 * (x - a)^2 + (y - b)^2 + (z - c)^2 = r^2. Notice how it is still squared and
 * _not_ cubed!
 * \param v Vector to test
 * \param c Vector containing the centre of the sphere
 * \param r The radius of the sphere
 * \return If v falls within the sphere
 */
static inline bool IntVector3_InSphere(IntVector3 v, IntVector3 c, unsigned r)
{
IntVector3 delta = v - c;
// Explictily cast to "unsigned int" because this number never can be
// negative, due to the fact that these numbers are squared. Still GCC
// warns about a comparison of a comparison between an unsigned and a
// signed integer.
return (unsigned int)dot(delta, delta) < r * r;
}