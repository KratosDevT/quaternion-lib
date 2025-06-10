// TestLibProject.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "FinalProject.h"
#include <assert.h>
#include "TestLibProject.h"


void customAssert(bool condition, const std::string& testName) {
    if (condition) {
        std::cout << "TEST PASS: " << testName << std::endl;
    }
    else {
        std::cout << "TEST FAIL: " << testName << std::endl;
        assert(false);
    }
}
void TestQuaternionClass();
void TestVectorClass();

int main()
{
    //TestVectorClass();
    TestQuaternionClass();    
}


void TestQuaternionClass() 
{
    Graphics::Quaternion quaternion = Graphics::Quaternion(1, 1, 2, 3);
    std::cout << "Quaternione: ";
    printQuaternion(quaternion);
    std::cout << std::endl;

    std::cout << "norma quadra: " << quaternion.squaredNorm() << std::endl;
    customAssert(quaternion.squaredNorm() == 15, "norma quadra");
    std::cout << std::endl;

    std::cout << "norma: " << quaternion.norm() << std::endl;
    customAssert((quaternion.norm() - std::sqrt(15)) < FLT_EPSILON, "norma");
    std::cout << std::endl;

    Graphics::Vector3D vector = Graphics::Vector3D(-1, -1, -2);
    Graphics::Quaternion quaternion2 = Graphics::Quaternion(vector, 3);
    std::cout << "coniugato: ";
    printQuaternion(quaternion.conjugate());
    customAssert(quaternion.conjugate() == quaternion2, "coniugato");
    std::cout << std::endl;

    Graphics::Quaternion quaternion3 = quaternion * Graphics::Quaternion::IDENTITY;
    std::cout << "prodotto per identita: ";
    printQuaternion(quaternion3);
    customAssert(quaternion3 == quaternion, "prodotto per identita");
    std::cout << std::endl;

    Graphics::Quaternion sumquat = quaternion + quaternion.conjugate();
    std::cout << "somma coniugato: ";
    printQuaternion(sumquat);
    customAssert(sumquat.getRe() == 2 * quaternion.getRe(), "somma coniugato, parte reale");
    customAssert(sumquat.getImg() == Graphics::Vector3D::ORIGIN, "somma coniugato, parte immaginaria");
    std::cout << std::endl;

}

void TestVectorClass()
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
