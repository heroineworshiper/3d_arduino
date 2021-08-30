#!/usr/bin/python

#
# 3D animated graphics over the REGIS protocol
#
# Copyright (C) 2021 Adam Williams <broadcast at earthling dot net>
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
# 






# generate models for 3d_arduino
# ./genmodel.py > models.h

import math


class Vector:
    def __init__(self, x, y, z, beginPoly):
        self.x = x;
        self.y = y;
        self.z = z;
        self.beginPoly = beginPoly

def polToVec(r, a, z, beginPoly):
    return Vector(r * math.cos(a), r * math.sin(a), z, beginPoly)

def makeGear():
    teeth = 12;
    r1 = 1.0;
    r2 = 1.2;
    l = 0.5;
    z0 = -l / 2.0;
    index = 0;
    shaft_l = 0.2
    shaft_r = 0.1
    output = []

    bevel = math.pi * 2 / teeth / 2 / 8
    # 2 faces
    for j in range(0, 2):
        z = z0 + l * j
        beginPoly = 1
        for i in range(0, teeth):
            angle1 = i * 2 * math.pi / teeth
            angle2 = (i + .5) * 2 * math.pi / teeth
            angle3 = (i + 1) * 2 * math.pi / teeth
            output.append(polToVec(r1, angle1 + bevel, z, beginPoly))
            output.append(polToVec(r1, angle2 - bevel, z, 0))
            output.append(polToVec(r2, angle2 + bevel, z, 0))
            output.append(polToVec(r2, angle3 - bevel, z, 0))
            beginPoly = 0
        output.append(polToVec(r1, bevel, z, 0))
    # joining lines
    for i in range(0, teeth):
        angle1 = i * 2 * math.pi / teeth
        angle2 = (i + .5) * 2 * math.pi / teeth
        output.append(polToVec(r1, angle1 + bevel, z0, 1))
        output.append(polToVec(r1, angle1 + bevel, z0 + l, 0))
        output.append(polToVec(r2, angle1 - bevel, z0, 1))
        output.append(polToVec(r2, angle1 - bevel, z0 + l, 0))
        output.append(polToVec(r1, angle2 - bevel, z0, 1))
        output.append(polToVec(r1, angle2 - bevel, z0 + l, 0))
        output.append(polToVec(r2, angle2 + bevel, z0, 1))
        output.append(polToVec(r2, angle2 + bevel, z0 + l, 0))


    # shaft
    # near plane
    output.append(Vector(-shaft_r, -shaft_r, z0, 1))
    output.append(Vector(shaft_r, -shaft_r, z0, 0))
    output.append(Vector(shaft_r, shaft_r, z0, 0))
    output.append(Vector(-shaft_r, shaft_r, z0, 0))
    output.append(Vector(-shaft_r, -shaft_r, z0, 0))
    # far plane
    output.append(Vector(-shaft_r, -shaft_r, z0 + l, 1))
    output.append(Vector(shaft_r, -shaft_r, z0 + l, 0))
    output.append(Vector(shaft_r, shaft_r, z0 + l, 0))
    output.append(Vector(-shaft_r, shaft_r, z0 + l, 0))
    output.append(Vector(-shaft_r, -shaft_r, z0 + l, 0))
    # join planes
    output.append(Vector(-shaft_r, -shaft_r, z0, 1))
    output.append(Vector(-shaft_r, -shaft_r, z0 + l, 0))
    output.append(Vector(shaft_r, -shaft_r, z0, 1))
    output.append(Vector(shaft_r, -shaft_r, z0 + l, 0))
    output.append(Vector(shaft_r, shaft_r, z0, 1)) 
    output.append(Vector(shaft_r, shaft_r, z0 + l, 0))
    output.append(Vector(-shaft_r, shaft_r, z0, 1)) 
    output.append(Vector(-shaft_r, shaft_r, z0 + l, 0))
    return output


def makeIcos():
    output = []
    r = 1.0
    pitch1 = math.pi * 1 / 3;
    pitch2 = math.pi * 2 / 3;
    r1 = r * math.sin(pitch1)
    r2 = r * math.sin(pitch2);
    z1 = r * math.cos(pitch1);
    z2 = r * math.cos(pitch2);

# 1 pass for each polyline
    for i in range(0, 4):
        for hdivision in range(0, 6):
            yaw1 = hdivision * math.pi * 2 / 5
            yaw2 = (hdivision + 1) * math.pi * 2 / 5
            yaw3 = (hdivision + 0.5) * math.pi * 2 / 5
            yaw4 = (hdivision + 1.5) * math.pi * 2 / 5
            if i == 0:
                output.append(Vector(0, 0, r, 1))
                output.append(polToVec(r1, yaw1, z1, 0))
                output.append(Vector(0, 0, -r, 1))
                output.append(polToVec(r2, yaw3, z2, 0))
            if i == 1:
                if hdivision == 0:
                    output.append(polToVec(r1, yaw1, z1, 1))
                output.append(polToVec(r1, yaw2, z1, 0))
            if i == 2:
                if hdivision == 0:
                    output.append(polToVec(r2, yaw3, z2, 1))
                output.append(polToVec(r2, yaw4, z2, 0))
            if i == 3:
                if hdivision == 0:
                    output.append(polToVec(r1, yaw1, z1, 1))
                output.append(polToVec(r2, yaw3, z2, 0))
                output.append(polToVec(r1, yaw2, z1, 0))


    return output



def makeBox():
    output = []
# near plane
    output.append(Vector(-1, -1, -1, 1))
    output.append(Vector(1, -1, -1, 0))
    output.append(Vector(1, 1, -1, 0))
    output.append(Vector(-1, 1, -1, 0))
    output.append(Vector(-1, -1, -1, 0))
# far plane
    output.append(Vector(-1, -1, 1, 1))
    output.append(Vector(1, -1, 1, 0))
    output.append(Vector(1, 1, 1, 0))
    output.append(Vector(-1, 1, 1, 0))
    output.append(Vector(-1, -1, 1, 0))
# join planes
    output.append(Vector(-1, -1, -1, 1))
    output.append(Vector(-1, -1, 1, 0))
    output.append(Vector(1, -1, -1, 1))
    output.append(Vector(1, -1, 1, 0))
    output.append(Vector(1, 1, -1, 1))
    output.append(Vector(1, 1, 1, 0))
    output.append(Vector(-1, 1, -1, 1))
    output.append(Vector(-1, 1, 1, 0))
    return output


def printModel(output, title):
    print("const point_t %s[] PROGMEM =" % title)
    print("{")
    for i in range(0, len(output)):
        print("    { %f, %f, %f, %d }," % (output[i].x, output[i].y, output[i].z, output[i].beginPoly))
    print("};")
    print("// count=%d bytes=%d\n\n" % (len(output), len(output) * (4 * 3 + 1)))
    



print("typedef struct");
print("{");
print("    float x, y, z;")
print("    unsigned char begin_poly;")
print("} point_t;" )


output = makeGear()
printModel(output, "gear")

output = makeBox()
printModel(output, "box")

output = makeIcos()
printModel(output, "icos")



