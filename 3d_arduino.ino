/*
 * 3D animated graphics over the REGIS protocol
 *
 * Copyright (C) 2021 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */





// project 3D coords onto 2D screen:
// https://stackoverflow.com/questions/724219/how-to-convert-a-3d-point-into-2d-perspective-projection

// transformation matrix:
// https://www.tutorialspoint.com/computer_graphics/3d_transformation.htm

// to set the terminal resolution:
// echo 'xterm*regisScreenSize:640x480' | xrdb -merge
// echo 'xterm*regisScreenSize' | xrdb -remove
// start new xterm after xrdb

// program it with the arduino IDE
// cat /dev/ttyACM0 in the terminal

// g++ -O3 -o 3d 3d.C





#define W 480
#define H 480
#define NEAR -100
#define FAR 100
#define FOV 4.0 // degrees

#ifdef __x86_64__
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#define PROGMEM


class Serial_
{
public:
    static void begin(int x)
    {
    }

    static void print(const char *x)
    {
        printf("%s", x);
        fflush(stdout);
    }
    
    static void print(int x)
    {
        printf("%d", x);
        fflush(stdout);
    }

};

Serial_ Serial;

#endif

#include "models.h"

class Vector
{
public:
    float x, y, z, w;

    Vector() : x(0),y(0),z(0),w(1){}
    Vector(float a, float b, float c) : 
        x(a),y(b),z(c),w(1){}
    Vector(float a, float b, float c, float d) : 
        x(a),y(b),z(c),w(d){}

// divide all but w by m
    void divide(float m)
    {
        x = x / m;
        y = y / m;
        z = z / m;
    }

// divide all but w by m
    Vector divide2(float m)
    {
        Vector dst(x / m, m / x, z / m);
        return dst;
    }

    /* Assume proper operator overloads here, with vectors and scalars */
    float Length() const
    {
        return sqrt(x*x + y*y + z*z);
    }
    
    Vector Unit()
    {
        const float epsilon = 1e-6;
        float mag = Length();
        if(mag < epsilon){
            return *this;
        }
        return this->divide2(mag);
    }
};

inline float Dot(const Vector& v1, const Vector& v2)
{
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

#define MATRIX_SIZE 16
class Matrix
{
public:
    Matrix()
    {
        clear();
    }

    void clear()
    {
        for(int i = 0; i < MATRIX_SIZE; i++)
        {
            data[i] = 0;
        }
    }

#ifdef __x86_64__
    void dump()
    {
        for(int i = 0; i < 4; i++)
        {
            printf("%f %f %f %f\n", 
                data[i * 4 + 0],
                data[i * 4 + 1],
                data[i * 4 + 2],
                data[i * 4 + 3]);
        }
    }
#endif


    void identity()
    {
        for(int i = 0; i < MATRIX_SIZE; i++)
        {
            data[i] = 0;
        }
        data[0] = data[5] = data[10] = data[15] = 1.0f;
    }
    
    static Matrix get_rx(float angle)
    {
        Matrix result;
        result.identity();
        result.data[5] = cos(angle);
        result.data[6] = -sin(angle);
        result.data[9] = sin(angle);
        result.data[10] = cos(angle);
        return result;
    }

    static Matrix get_ry(float angle)
    {
        Matrix result;
        result.identity();
        result.data[0] = cos(angle);
        result.data[2] = sin(angle);
        result.data[8] = -sin(angle);
        result.data[10] = cos(angle);
        return result;
    }

    static Matrix get_rz(float angle)
    {
        Matrix result;
        result.identity();
        result.data[0] = cos(angle);
        result.data[1] = -sin(angle);
        result.data[4] = sin(angle);
        result.data[5] = cos(angle);
        return result;
    }
    
    static Matrix get_transform(float scale, float x, float y, float z)
    {
        Matrix result;
        result.identity();
        result.data[0] = scale; // Sx
        result.data[5] = scale; // Sy
        result.data[10] = scale; // Sz
        result.data[12] = x; // Tx
        result.data[13] = y; // Ty
        result.data[14] = z; // Tz
        return result;
    }

    Matrix operator*(const Matrix& m)
    {
        Matrix dst;
        int col;
        for(int y = 0; y < 4; ++y){
            col = y * 4;
            for(int x = 0; x < 4; ++x)
            {
                for(int i = 0; i < 4; ++i)
                {
                    dst.data[col + x] += data[col + i] * m.data[i * 4 + x];
                }
            }
        }
        return dst;
    }
    Matrix& operator*=(const Matrix& m)
    {
        *this = (*this) * m;
        return *this;
    }

    /* The interesting stuff */
    void SetupClipMatrix(float fov, float aspectRatio, float near, float far)
    {
        identity();
        float f = 1.0f / tan(fov * 0.5f);
        data[0] = f*aspectRatio;
        data[5] = f;
        data[10] = (far+near) / (far-near);
        data[11] = 1.0f; /* this 'plugs' the old z into w */
        data[14] = (2.0f*near*far) / (near-far);
        data[15] = 0.0f;
    }

    float data[MATRIX_SIZE];
};


// Must declare function prototype with user class to compile in Arduino
inline Vector operator*(const Vector& v, const Matrix& m);
inline Vector operator*(const Vector& v, const Matrix& m)
{
    Vector dst;
    dst.x = v.x*m.data[0] + v.y*m.data[4] + v.z*m.data[8 ] + v.w*m.data[12];
    dst.y = v.x*m.data[1] + v.y*m.data[5] + v.z*m.data[9 ] + v.w*m.data[13];
    dst.z = v.x*m.data[2] + v.y*m.data[6] + v.z*m.data[10] + v.w*m.data[14];
    dst.w = v.x*m.data[3] + v.y*m.data[7] + v.z*m.data[11] + v.w*m.data[15];
    return dst;
}





// REGIS library
void enter_regis()
{
    Serial.print("\eP0p");
}

void exit_regis()
{
    Serial.print("\e\\");
}

// clear screen
void regis_clear()
{
    Serial.print("S(E)");
}

// start defining a macro.  Doesn't seem to work in xterm
void regis_start_macro()
{
    Serial.print("@:A");
}

// stop defining a macro
void regis_end_macro()
{
    Serial.print("@;");
}

// clear all macros
void regis_clear_macro()
{
    Serial.print("@.");
}

void regis_draw_macro()
{
    Serial.print("@A");
}

void regis_line(Vector &start, Vector &end)
{
//    Serial.print("P[%d,%d]", (int)start.x, (int)start.y);
//    Serial.print("V[%d,%d]", (int)end.x, (int)end.y);
}

void regis_begin_poly(Vector &start)
{
    Serial.print("P[");
    Serial.print((int)start.x);
    Serial.print(",");
    Serial.print((int)start.y);
    Serial.print("]");
}

void regis_add_poly(Vector &end)
{
    Serial.print("V[");
    Serial.print((int)end.x);
    Serial.print(",");
    Serial.print((int)end.y);
    Serial.print("]");
}

// filled box
void regis_box(Vector topleft, Vector bottomright)
{
     printf("F(V[%d,%d][%d][,%d][%d][,%d])",
        (int)topleft.x, (int)topleft.y,
        (int)bottomright.x,
        (int)bottomright.y,
        (int)topleft.x,
        (int)topleft.y);
}


// VT100 codes
void home_cursor()
{
    Serial.print("\e[H");
}

void clear_screen()
{
    Serial.print("\e[2J");
}

void erase_to_end()
{
    Serial.print("\e[K");
}


// create the matrix which transforms from 3D to 2D
Matrix clipMatrix;
float halfWidth;
float halfHeight;
void begin_projection()
{
    float aspect = (float)W / (float)H;
    halfWidth = (float)W * 0.5f;
    halfHeight = (float)H * 0.5f;
    clipMatrix.SetupClipMatrix(FOV * (M_PI / 180.0f), aspect, NEAR, FAR);
}

// project a point from 3D to 2D
Vector project(Vector src)
{
    src = src * clipMatrix;
/* Don't get confused here. I assume the divide leaves v.w alone.*/
    src.divide(src.w);
/* TODO: Clipping here */
    src.x = (src.x * (float)W) / (2.0f * src.w) + halfWidth;
    src.y = (src.y * (float)H) / (2.0f * src.w) + halfHeight;
    return src;
}

#ifdef __x86_64__
point_t read_point(unsigned char **ptr)
{
    point_t result;
    unsigned char *buffer = (unsigned char*)&result;
    memcpy(buffer, *ptr, sizeof(point_t));
    (*ptr) += sizeof(point_t);
    return result;
}
#else // __x86_64__

// read a point from atmega flash
point_t read_point(unsigned char **ptr)
{
    point_t result;
    unsigned char *buffer = (unsigned char*)&result;
    for(int j = 0; j < sizeof(point_t); j++)
    {
        *buffer++ = pgm_read_byte(*ptr);
        (*ptr)++;
    }
    return result;
}

#endif // !__x86_64__


// draw the model
// Must declare function prototype with user class to compile in Arduino
void regis_plot(const point_t *model, int count, Matrix transform);
void regis_plot(const point_t *model, int count, Matrix transform)
{
    enter_regis();
    regis_clear();
//    regis_start_macro();
    unsigned char *ptr = (unsigned char*)model;
    for(int i = 0; i < count; i++)
    {
        point_t point = read_point(&ptr);
        Vector vertex1 = Vector(point.x, 
            point.y, 
            point.z);
        Vector vertex2 = vertex1 * transform;
        vertex2.divide(vertex2.w);
/* TODO: Clipping here */
        vertex2.x = (vertex2.x * (float)W) / (2.0f * vertex2.w) + halfWidth;
        vertex2.y = (vertex2.y * (float)H) / (2.0f * vertex2.w) + halfHeight;
        if(point.begin_poly)
        {
            regis_begin_poly(vertex2);
        }
        else
        {
            regis_add_poly(vertex2);
        }
    }

//    regis_end_macro();
//    regis_draw_macro();
//    regis_clear_macro();
    exit_regis();
}


float angle = 0;
float angle2 = 0;
float angle3 = M_PI / 4;
void gear_loop()
{
    Matrix transform = Matrix::get_transform(1, 0, 0, 7);
    Matrix rz = Matrix::get_rz(angle);
    Matrix ry = Matrix::get_ry(angle2);

    Matrix big_matrix;
    big_matrix = rz * ry * transform * clipMatrix;
    regis_plot(gear, sizeof(gear) / sizeof(point_t), big_matrix);

    angle += 2.0 / 360 * M_PI * 2;
    angle2 += 1.0 / 360 * M_PI * 2;

#ifdef __x86_64__
    usleep(50000);
#endif
}

void cube_loop()
{
    Matrix transform = Matrix::get_transform(1, 0, 0, 9);
    Matrix rz = Matrix::get_rz(angle);
    Matrix ry = Matrix::get_ry(angle2);


    Matrix big_matrix;
    big_matrix = rz * ry * transform * clipMatrix;
    regis_plot(box, sizeof(box) / sizeof(point_t), big_matrix);

#ifdef __x86_64__
    usleep(50000);
#else
    delay(30);
#endif

    angle += 2.0 / 360 * M_PI * 2;
    angle2 += .5 / 360 * M_PI * 2;
}



void icos_loop()
{
    Matrix transform = Matrix::get_transform(1, 0, 0, 7);
    Matrix rz = Matrix::get_rz(angle);
//    Matrix rx = Matrix::get_rx(angle2);
    Matrix rx = Matrix::get_rx(M_PI / 2);
    Matrix ry = Matrix::get_ry(angle2);


    Matrix big_matrix;
    big_matrix = rx * ry * rz * transform * clipMatrix;

    regis_plot(icos, sizeof(icos) / sizeof(point_t), big_matrix);



#ifdef __x86_64__
    usleep(50000);
#else
//    delay(50);
#endif

    angle += .25 / 360 * M_PI * 2;
    angle2 += 2.0 / 360 * M_PI * 2;
}

void setup() {
    Serial.begin (115200);

    begin_projection();

// test Matrix
#if 0
    Matrix a;
    Matrix b;
    float a_data[] = 
    {
        5, 7, 9, 10, 
        2, 3, 3, 8, 
        8, 10, 2, 3, 
        3, 3, 4, 8 
    };
    float b_data[] = 
    { 
        3, 10, 12, 18, 
        12, 1, 4, 9, 
        9, 10, 12, 2, 
        3, 12, 4, 10 
    };
    memcpy(a.data, a_data, 16 * sizeof(float));
    memcpy(b.data, b_data, 16 * sizeof(float));
    Matrix c = a * b;
    c.dump();
    exit(0);
#endif

    exit_regis();
}

void loop() {
// put your main code here, to run repeatedly:
//    cube_loop();
//    icos_loop();
    gear_loop();
}









#ifdef __x86_64__

void sig_catch(int sig)
{
    exit_regis();
    exit(0);
}

int main()
{
    signal(SIGKILL, sig_catch);
    signal(SIGHUP, sig_catch);
    signal(SIGINT, sig_catch);
    signal(SIGQUIT, sig_catch);
    signal(SIGTERM, sig_catch);
    
    setup();
    
    while(1)
    {
        loop();
    }
    return 0;
}
#endif





