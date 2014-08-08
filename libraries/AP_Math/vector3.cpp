/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * vector3.cpp
 * Copyright (C) Andrew Tridgell 2012
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Math.h"

#define HALF_SQRT_2 0.70710678118654757f
#define COS_5       0.99619469809174553f
#define SIN_5       0.08715574274765817f
#define COS_10      0.98480775301220805f
#define SIN_10      0.17364817766693034f
#define COS_15      0.96592582628906829f
#define SIN_15      0.25881904510252076f
#define COS_20      0.93969262078590838f
#define SIN_20		0.34202014332566873f
#define COS_25      0.90630778703664996f
#define SIN_25      0.42261826174069943f
#define COS_30      0.86602540378443864f
#define SIN_30      0.50000000000000000f
#define COS_35      0.81915204428899178f
#define SIN_35      0.57357643635104609f
#define COS_40      0.76604444311897803f
#define SIN_40      0.64278760968653932f
#define COS_45 		HALF_SQRT_2
#define SIN_45      HALF_SQRT_2
#define COS_50      SIN_40
#define SIN_50      COS_40
#define COS_55      SIN_35
#define SIN_55      COS_35
#define COS_60      SIN_30
#define SIN_60      COS_30
#define COS_65      SIN_25
#define SIN_65      COS_25
#define COS_70      SIN_20
#define SIN_70      COS_20
#define COS_75      SIN_15
#define SIN_75      COS_15
#define COS_80      SIN_10
#define SIN_80      COS_10
#define COS_85      SIN_5
#define SIN_85      COS_5   


// rotate a vector by a standard rotation, attempting
// to use the minimum number of floating point operations
template <typename T>
void Vector3<T>::rotate(enum Rotation rotation)
{
    T tmp;
    switch (rotation) {
    case ROTATION_NONE:
    case ROTATION_MAX:
        return;
    case ROTATION_YAW_45: {
        tmp = HALF_SQRT_2*(x - y);
        y   = HALF_SQRT_2*(x + y);
        x = tmp;
        return;
    }
    case ROTATION_YAW_90: {
        tmp = x; x = -y; y = tmp;
        return;
    }
    case ROTATION_YAW_135: {
        tmp = -HALF_SQRT_2*(x + y);
        y   =  HALF_SQRT_2*(x - y);
        x = tmp;
        return;
    }
    case ROTATION_YAW_180:
        x = -x; y = -y;
        return;
    case ROTATION_YAW_225: {
        tmp = HALF_SQRT_2*(y - x);
        y   = -HALF_SQRT_2*(x + y);
        x = tmp;
        return;
    }
    case ROTATION_YAW_270: {
        tmp = x; x = y; y = -tmp;
        return;
    }
    case ROTATION_YAW_315: {
        tmp = HALF_SQRT_2*(x + y);
        y   = HALF_SQRT_2*(y - x);
        x = tmp;
        return;
    }
    case ROTATION_ROLL_180: {
        y = -y; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_45: {
        tmp = HALF_SQRT_2*(x + y);
        y   = HALF_SQRT_2*(x - y);
        x = tmp; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_90: {
        tmp = x; x = y; y = tmp; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_135: {
        tmp = HALF_SQRT_2*(y - x);
        y   = HALF_SQRT_2*(y + x);
        x = tmp; z = -z;
        return;
    }
    case ROTATION_PITCH_180: {
        x = -x; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_225: {
        tmp = -HALF_SQRT_2*(x + y);
        y   =  HALF_SQRT_2*(y - x);
        x = tmp; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_270: {
        tmp = x; x = -y; y = -tmp; z = -z;
        return;
    }
    case ROTATION_ROLL_180_YAW_315: {
        tmp =  HALF_SQRT_2*(x - y);
        y   = -HALF_SQRT_2*(x + y);
        x = tmp; z = -z;
        return;
    }
    case ROTATION_ROLL_90: {
        tmp = z; z = y; y = -tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_45: {
        tmp = z; z = y; y = -tmp;
        tmp = HALF_SQRT_2*(x - y);
        y   = HALF_SQRT_2*(x + y);
        x = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_90: {
        tmp = z; z = y; y = -tmp;
        tmp = x; x = -y; y = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_135: {
        tmp = z; z = y; y = -tmp;
        tmp = -HALF_SQRT_2*(x + y);
        y   =  HALF_SQRT_2*(x - y);
        x = tmp;
        return;
    }
    case ROTATION_ROLL_270: {
        tmp = z; z = -y; y = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_45: {
        tmp = z; z = -y; y = tmp;
        tmp = HALF_SQRT_2*(x - y);
        y   = HALF_SQRT_2*(x + y);
        x = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_90: {
        tmp = z; z = -y; y = tmp;
        tmp = x; x = -y; y = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_135: {
        tmp = z; z = -y; y = tmp;
        tmp = -HALF_SQRT_2*(x + y);
        y   =  HALF_SQRT_2*(x - y);
        x = tmp;
        return;
    }
    case ROTATION_PITCH_90: {
        tmp = z; z = -x; x = tmp;
        return;
    }
    case ROTATION_PITCH_270: {
        tmp = z; z = x; x = -tmp;
        return;
    }
    case ROTATION_PITCH_180_YAW_90: {
        z = -z;
        tmp = -x; x = -y; y = tmp;
        return;
    }
    case ROTATION_PITCH_180_YAW_270: {
        x = -x; z = -z;
        tmp = x; x = y; y = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_90: {
        tmp = z; z = y; y = -tmp;
        tmp = z; z = -x; x = tmp;
        return;
    }
    case ROTATION_ROLL_180_PITCH_90: {
        y = -y; z = -z;
        tmp = z; z = -x; x = tmp;
        return;
    }
    case ROTATION_ROLL_270_PITCH_90: {
        tmp = z; z = -y; y = tmp;
        tmp = z; z = -x; x = tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_180: {
        tmp = z; z = y; y = -tmp;
        x = -x; z = -z;
        return;
    }
    case ROTATION_ROLL_270_PITCH_180: {
        tmp = z; z = -y; y = tmp;
        x = -x; z = -z;
        return;
    }
    case ROTATION_ROLL_90_PITCH_270: {
        tmp = z; z = y; y = -tmp;
        tmp = z; z = x; x = -tmp;
        return;
    }
    case ROTATION_ROLL_180_PITCH_270: {
        y = -y; z = -z;
        tmp = z; z = x; x = -tmp;
        return;
    }
    case ROTATION_ROLL_270_PITCH_270: {
        tmp = z; z = -y; y = tmp;
        tmp = z; z = x; x = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_180_YAW_90: {
        tmp = z; z = y; y = -tmp;
        x = -x; z = -z;
        tmp = x; x = -y; y = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_270: {
        tmp = z; z = y; y = -tmp;
        tmp = x; x = y; y = -tmp;
        return;
    }
	case ROTATION_PITCH_5:	{ // from here (06/08/2014-Menno)
		tmp = COS_5*x + SIN_5*z;
		z = -SIN_5*x + COS_5*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_10:	{
		tmp = COS_10*x + SIN_10*z;
		z = -SIN_10*x + COS_10*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_15:	{
		tmp = COS_15*x + SIN_15*z;
		z = -SIN_15*x + COS_15*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_20:	{
		tmp = COS_20*x + SIN_20*z;
		z = -SIN_20*x + COS_20*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_25:	{
		tmp = COS_25*x + SIN_25*z;
		z = -SIN_25*x + COS_25*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_30:	{
		tmp = COS_30*x + SIN_30*z;
		z = -SIN_30*x + COS_30*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_35:	{
		tmp = COS_35*x + SIN_35*z;
		z = -SIN_35*x + COS_35*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_40:	{
		tmp = COS_40*x + SIN_40*z;
		z = -SIN_40*x + COS_40*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_45:	{
		tmp = COS_45*x + SIN_45*z;
		z = -SIN_45*x + COS_45*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_50:	{
		tmp = COS_50*x + SIN_50*z;
		z = -SIN_50*x + COS_50*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_55:	{
		tmp = COS_55*x + SIN_55*z;
		z = -SIN_55*x + COS_55*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_60:	{
		tmp = COS_60*x + SIN_60*z;
		z = -SIN_60*x + COS_60*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_65:	{
		tmp = COS_65*x + SIN_65*z;
		z = -SIN_65*x + COS_65*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_70:	{
		tmp = COS_70*x + SIN_70*z;
		z = -SIN_70*x + COS_70*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_75:	{
		tmp = COS_75*x + SIN_75*z;
		z = -SIN_75*x + COS_75*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_80:	{
		tmp = COS_80*x + SIN_80*z;
		z = -SIN_80*x + COS_80*z;
		x = tmp;
		return;
	}
	case ROTATION_PITCH_85:	{ 
		tmp = COS_85*x + SIN_85*z;
		z = -SIN_85*x + COS_85*z;
		x = tmp;
		return;
	} // to here (06/08/2014-Menno)
    }
}

// vector cross product
template <typename T>
Vector3<T> Vector3<T>::operator %(const Vector3<T> &v) const
{
    Vector3<T> temp(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    return temp;
}

// dot product
template <typename T>
T Vector3<T>::operator *(const Vector3<T> &v) const
{
    return x*v.x + y*v.y + z*v.z;
}

template <typename T>
float Vector3<T>::length(void) const
{
    return pythagorous3(x, y, z);
}

template <typename T>
Vector3<T> &Vector3<T>::operator *=(const T num)
{
    x*=num; y*=num; z*=num;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator /=(const T num)
{
    x /= num; y /= num; z /= num;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator -=(const Vector3<T> &v)
{
    x -= v.x; y -= v.y; z -= v.z;
    return *this;
}

template <typename T>
bool Vector3<T>::is_nan(void) const
{
    return isnan(x) || isnan(y) || isnan(z);
}

template <typename T>
bool Vector3<T>::is_inf(void) const
{
    return isinf(x) || isinf(y) || isinf(z);
}

template <typename T>
Vector3<T> &Vector3<T>::operator +=(const Vector3<T> &v)
{
    x+=v.x; y+=v.y; z+=v.z;
    return *this;
}

template <typename T>
Vector3<T> Vector3<T>::operator /(const T num) const
{
    return Vector3<T>(x/num, y/num, z/num);
}

template <typename T>
Vector3<T> Vector3<T>::operator *(const T num) const
{
    return Vector3<T>(x*num, y*num, z*num);
}

template <typename T>
Vector3<T> Vector3<T>::operator -(const Vector3<T> &v) const
{
    return Vector3<T>(x-v.x, y-v.y, z-v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator +(const Vector3<T> &v) const
{
    return Vector3<T>(x+v.x, y+v.y, z+v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator -(void) const
{
    return Vector3<T>(-x,-y,-z);
}

template <typename T>
bool Vector3<T>::operator ==(const Vector3<T> &v) const
{
    return (x==v.x && y==v.y && z==v.z);
}

template <typename T>
bool Vector3<T>::operator !=(const Vector3<T> &v) const
{
    return (x!=v.x && y!=v.y && z!=v.z);
}

template <typename T>
float Vector3<T>::angle(const Vector3<T> &v2) const
{
    return acosf(((*this)*v2) / (this->length()*v2.length()));
}

// multiplication of transpose by a vector
template <typename T>
Vector3<T> Vector3<T>::operator *(const Matrix3<T> &m) const
{
    return Vector3<T>(*this * m.colx(),
                      *this * m.coly(),
                      *this * m.colz());
}

// multiply a column vector by a row vector, returning a 3x3 matrix
template <typename T>
Matrix3<T> Vector3<T>::mul_rowcol(const Vector3<T> &v2) const
{
    const Vector3<T> v1 = *this;
    return Matrix3<T>(v1.x * v2.x, v1.x * v2.y, v1.x * v2.z,
                      v1.y * v2.x, v1.y * v2.y, v1.y * v2.z,
                      v1.z * v2.x, v1.z * v2.y, v1.z * v2.z);
}

// only define for float
template void Vector3<float>::rotate(enum Rotation);
template float Vector3<float>::length(void) const;
template Vector3<float> Vector3<float>::operator %(const Vector3<float> &v) const;
template float Vector3<float>::operator *(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator *(const Matrix3<float> &m) const;
template Matrix3<float> Vector3<float>::mul_rowcol(const Vector3<float> &v) const;
template Vector3<float> &Vector3<float>::operator *=(const float num);
template Vector3<float> &Vector3<float>::operator /=(const float num);
template Vector3<float> &Vector3<float>::operator -=(const Vector3<float> &v);
template Vector3<float> &Vector3<float>::operator +=(const Vector3<float> &v);
template Vector3<float> Vector3<float>::operator /(const float num) const;
template Vector3<float> Vector3<float>::operator *(const float num) const;
template Vector3<float> Vector3<float>::operator +(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator -(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator -(void) const;
template bool Vector3<float>::operator ==(const Vector3<float> &v) const;
template bool Vector3<float>::operator !=(const Vector3<float> &v) const;
template bool Vector3<float>::is_nan(void) const;
template bool Vector3<float>::is_inf(void) const;
template float Vector3<float>::angle(const Vector3<float> &v) const;
