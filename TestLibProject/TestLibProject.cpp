// TestLibProject.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "FinalProject.h"
#include <assert.h>
#include "TestLibProject.h"

int main()
{
    //TestsVectorImpl();
    Graphics::Quaternion quaternion = Graphics::Quaternion(1, 2, 3, 4);
    printQuaternion(quaternion);
    
    
}

void TestsVectorImpl()
{
    Graphics::Vector3D vec1 = Graphics::Vector3D::RIGHT;
    Graphics::Vector3D vec2 = Graphics::Vector3D::UP;

    Graphics::Vector3D sumtest(1, 1, 0);
    Graphics::Scalar dottest = 0;
    Graphics::Vector3D crosstest(0, 0, 1);

    assert((vec1 + vec2) == sumtest);
    assert((dot(vec1, vec2) - dottest) <= FLT_EPSILON);
    assert(cross(vec1, vec2) == crosstest);
    assert(dot(vec1, vec1) == vec1.squaredNorm());
    Graphics::Vector3D versor = (vec1 + vec2).normalize();
    assert((versor.norm() - 1) <= FLT_EPSILON);

    printVector3D(versor);
}
