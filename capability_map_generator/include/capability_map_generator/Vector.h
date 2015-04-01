// Copyright (c) 2014, Jochen Kempfle
// All rights reserved.

/*
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/


#ifndef VECTOR_H
#define VECTOR_H

namespace capability_map_generator
{

// small helper class used for coordinates and vectors
class Vector
{
  public:
    Vector() { }
    ~Vector() { }
    Vector(double xVal, double yVal, double zVal) : x(xVal), y(yVal), z(zVal) { }
    Vector(const Vector &rhs) : x(rhs.x), y(rhs.y), z(rhs.z) { }
    
    inline Vector operator*(double rhs) const { return Vector(x * rhs, y * rhs, z * rhs); }
    inline Vector operator+(const Vector &rhs) const { return Vector(x + rhs.x, y + rhs.y, z + rhs.z); }
    
    inline Vector cross(const Vector &rhs) const 
    {
        return Vector(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x);
    }
    
    inline double dot(const Vector &rhs) const 
    {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }
    
    double x, y, z;
};

}

#endif // VECTOR_H
