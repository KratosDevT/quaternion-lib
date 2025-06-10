// TestLibProject.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "FinalProject.h"
#include <assert.h>
#include "TestLibProject.h"
#include <cmath>

const double PI = 4.0 * atan(1.0);

void customAssert(bool condition, const std::string& testName) {
    if (condition) {
        std::cout << "TEST PASS: " << testName << std::endl;
    }
    else {
        std::cout << "TEST FAIL: " << testName << std::endl;
        assert(false);
    }
}
void TestQuaternionClass(Graphics::Quaternion quaternion, Graphics::Quaternion conjugate, Graphics::Scalar squaredNorm);
void TestVectorClass();

int main()
{
    TestVectorClass();
    Graphics::Quaternion quaternion = Graphics::Quaternion(1, 2, 3, 4);
    Graphics::Quaternion conjugate = Graphics::Quaternion(-1, -2, -3, 4);
    Graphics::Scalar squaredNorm = 30;
    TestQuaternionClass(quaternion, conjugate, squaredNorm);


	//test asse angolo
    Graphics::Vector3D vec = Graphics::Vector3D(1, 1, 0);

    Graphics::Vector3D axis = Graphics::Vector3D(1, -1, 0);
    Graphics::Scalar angle = PI / 2; // Angolo in rad
	//std::cout << "pigreco mezzi 1,57: " << angle << std::endl;
    Graphics::Quaternion quaternionRotator = Graphics::Quaternion(axis, angle, Graphics::QuaternionType::FROM_AXIS_ANGLE);

    std::cout << "rotazione vettore: ";
    printVector3D(vec);
    std::cout << "con Quaternione: ";
    printQuaternion(quaternionRotator);

    Graphics::Vector3D rotated = quaternionRotator.rotate(vec);
    std::cout << "vettore ruotato: ";
    printVector3D(rotated);
    customAssert(rotated == Graphics::Vector3D::FORWARD * std::sqrt(2), "quaternione di rotazione da asse angolo");

}


void TestQuaternionClass(Graphics::Quaternion quaternion, Graphics::Quaternion conjugate, Graphics::Scalar squaredNorm)
{
    std::cout << "Quaternione: ";
    printQuaternion(quaternion);
    std::cout << std::endl;

    std::cout << "norma quadra: " << quaternion.squaredNorm() << std::endl;
    customAssert(quaternion.squaredNorm() == squaredNorm, "norma quadra");
    std::cout << std::endl;

    std::cout << "norma: " << quaternion.norm() << std::endl;
    customAssert((quaternion.norm() - std::sqrt(squaredNorm)) < FLT_EPSILON, "norma");
    std::cout << std::endl;

    std::cout << "normalizzazione: ";
    printQuaternion(quaternion.normalize()); 
    customAssert((quaternion.normalize().norm() - 1.0f) < FLT_EPSILON, "normalizzazione");
    std::cout << std::endl;

    std::cout << "coniugato: ";
    printQuaternion(quaternion.conjugate());
    customAssert(quaternion.conjugate() == conjugate, "coniugato");
    std::cout << std::endl;

    Graphics::Quaternion quaternion3 = quaternion * Graphics::Quaternion::IDENTITY;
    std::cout << "prodotto per identita: ";
    printQuaternion(quaternion3);
    customAssert(quaternion3 == quaternion, "prodotto per identita");
    std::cout << std::endl;

    Graphics::Quaternion sumquat = quaternion + quaternion;
    std::cout << "somma con se stesso: ";
    printQuaternion(sumquat);
    customAssert(sumquat == 2 * quaternion, "somma con se stesso");
    std::cout << std::endl;

    Graphics::Quaternion sumconj = quaternion + quaternion.conjugate();
    std::cout << "somma con coniugato: ";
    printQuaternion(sumconj);
    customAssert(sumconj.getRe() == 2 * quaternion.getRe(), "somma con coniugato, parte reale");
    customAssert(sumconj.getImg() == Graphics::Vector3D::ORIGIN, "somma con coniugato, parte immaginaria");
    std::cout << std::endl;

    Graphics::Quaternion diffquat = quaternion - quaternion;
    std::cout << "differenza con se stesso: ";
    printQuaternion(diffquat);
    customAssert(diffquat == Graphics::Quaternion::ZERO, "differenza con se stesso");
    std::cout << std::endl;

    Graphics::Quaternion diffconj = quaternion - quaternion.conjugate();
    std::cout << "differenza con coniugato: ";
    printQuaternion(diffconj);
    customAssert(diffconj.getRe() == 0, "differenza con coniugato, parte reale");
    customAssert(diffconj.getImg() == 2 * quaternion.getImg(), "differenza con coniugato, parte immaginaria");
    std::cout << std::endl;

    Graphics::Quaternion inverse = quaternion.inverse();
    std::cout << "inverso: ";
    printQuaternion(inverse);
    Graphics::Quaternion prodinverse = quaternion * inverse;
    std::cout << "prodotto per inverso: ";
    printQuaternion(prodinverse);
    customAssert(prodinverse == Graphics::Quaternion::IDENTITY, "prodotto per inverso");
    std::cout << std::endl;

    
    Graphics::Vector3D vec = Graphics::Vector3D::RIGHT;
    Graphics::Quaternion quaternionRotator = Graphics::Quaternion(0, -std::sqrt(2) / 2, 0, std::sqrt(2) / 2);
    std::cout << "rotazione di [1,0,0] con Quaternione: ";
    printQuaternion(quaternionRotator);
    Graphics::Vector3D rotated = quaternionRotator.rotate(vec);
    std::cout << "rotazione risultante: ";
    printVector3D(rotated);
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
