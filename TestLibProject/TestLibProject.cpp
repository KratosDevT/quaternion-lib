// TestLibProject.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "FinalProject.h"
#include <assert.h>
#include "TestLibProject.h"
#include <cmath>

const double PI = 4.0 * atan(1.0);

void TestQuaternionClass(Graphics::Quaternion quaternion, Graphics::Quaternion conjugate, Graphics::Scalar squaredNorm);
void TestRotazioneAsseAngolo(Graphics::Vector3D& vec);
void TestRotazioneIdentitaria(Graphics::Vector3D& vec);
void TestRotazioneSLerp(Graphics::Vector3D& vec2, Graphics::Quaternion& quaternionRotator2, Graphics::Quaternion& quaternionRotator3);
void TestRotazioneNLerp(Graphics::Vector3D& vec2, Graphics::Quaternion& quaternionRotator2, Graphics::Quaternion& quaternionRotator3);
void TestVectorClass();

void customAssert(bool condition, const std::string& testName) {
    if (condition) {
        std::cout << "TEST PASS: " << testName << std::endl;
    }
    else {
        std::cout << "TEST FAIL: " << testName << std::endl;
        assert(false);
    }
}

int main()
{
    TestVectorClass();
    Graphics::Quaternion quaternion = Graphics::Quaternion(1, 2, 3, 4);
    Graphics::Quaternion conjugate = Graphics::Quaternion(-1, -2, -3, 4);
    Graphics::Scalar squaredNorm = 30;
    TestQuaternionClass(quaternion, conjugate, squaredNorm);

    Graphics::Vector3D vec1 = Graphics::Vector3D(-1, 0, 0);

    //test asse angolo
    TestRotazioneAsseAngolo(vec1);
    TestRotazioneIdentitaria(vec1);

    Graphics::Vector3D vec2 = Graphics::Vector3D(-1, 1, 0);

    Graphics::Vector3D axis = Graphics::Vector3D(1, 0, 0);
    Graphics::Scalar angle = PI;
    Graphics::Quaternion quaternionRotator1 = Graphics::Quaternion(axis, angle, Graphics::QuaternionType::FROM_AXIS_ANGLE);

    Graphics::Vector3D axis2 = Graphics::Vector3D(0, 1, 0);
    Graphics::Scalar angle2 = PI;
    Graphics::Quaternion quaternionRotator2 = Graphics::Quaternion(axis2, angle2, Graphics::QuaternionType::FROM_AXIS_ANGLE);

    TestRotazioneNLerp(vec2, quaternionRotator1, quaternionRotator2);
    TestRotazioneSLerp(vec2, quaternionRotator1, quaternionRotator2);
}

void TestRotazioneSLerp(Graphics::Vector3D& vec2,Graphics::Quaternion& quaternionRotator2, Graphics::Quaternion& quaternionRotator3)
{
    std::cout << "INTERPOLAZIONE SLERP";
    std::cout << std::endl;
    Graphics::Quaternion quatSlerp = Graphics::slerp(quaternionRotator2, quaternionRotator3, 0.5f);

    printQuaternion(quatSlerp);
    printQuaternionAxisAndAngle(quatSlerp);

    Graphics::Vector3D rotated3 = quatSlerp.rotate(vec2);
    std::cout << "vettore: ";
    printVector3D(vec2);
    std::cout << "vettore ruotato con quaternione interpolato slerp: ";
    printVector3D(rotated3);
    std::cout << std::endl;
    std::cout << std::endl;
}

void TestRotazioneNLerp(Graphics::Vector3D& vec2, Graphics::Quaternion& quaternionRotator2, Graphics::Quaternion& quaternionRotator3)
{
    std::cout << "INTERPOLAZIONE NLERP";
    std::cout << std::endl;
    Graphics::Quaternion quatNlerp = Graphics::nlerp(quaternionRotator2, quaternionRotator3, 0.5f);

    printQuaternion(quatNlerp);
    printQuaternionAxisAndAngle(quatNlerp);

    std::cout << "norma quatNlerp:";
    std::cout << quatNlerp.norm() << std::endl;
    Graphics::Vector3D rotated2 = quatNlerp.rotate(vec2);
    std::cout << "vettore: ";
    printVector3D(vec2);
    std::cout << "vettore ruotato con quaternione interpolato nlerp: ";
    printVector3D(rotated2);
    std::cout << std::endl;
    std::cout << std::endl;
}

void TestRotazioneIdentitaria(Graphics::Vector3D& vec)
{
    std::cout << "ROTAZIONE IDENTITARIA";
    std::cout << std::endl;
    std::cout << "vettore: ";
    printVector3D(vec);

    Graphics::Vector3D axis2 = Graphics::Vector3D(0, 0, 0);
    Graphics::Scalar angle2 = 0.0f;
    Graphics::Quaternion quaternionRotator2 = Graphics::Quaternion(axis2, angle2, Graphics::QuaternionType::FROM_AXIS_ANGLE);

    printQuaternion(quaternionRotator2);
    printQuaternionAxisAndAngle(quaternionRotator2);
    Graphics::Vector3D rotated2 = quaternionRotator2.rotate(vec);
    std::cout << "vettore ruotato con identita: ";
    printVector3D(rotated2);
    std::cout << std::endl;
    std::cout << std::endl;
}

void TestRotazioneAsseAngolo(Graphics::Vector3D& vec)
{
    std::cout << "Rotazione da Asse Angolo";
    std::cout << std::endl;
    Graphics::Vector3D axis = Graphics::Vector3D(0, 1, 0);
    Graphics::Scalar angle = PI;
    Graphics::Quaternion quaternionRotator = Graphics::Quaternion(axis, angle, Graphics::QuaternionType::FROM_AXIS_ANGLE);
    std::cout << "rotazione del vettore: ";
    printVector3D(vec);
    std::cout << "con Quaternione: ";
    printQuaternion(quaternionRotator);
    printQuaternionAxisAndAngle(quaternionRotator);
    Graphics::Vector3D rotated = quaternionRotator.rotate(vec);
    std::cout << "vettore ruotato: ";
    printVector3D(rotated);
    customAssert(rotated == Graphics::Vector3D::RIGHT, "quaternione di rotazione da asse angolo");
    std::cout << std::endl;
    std::cout << std::endl;
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
    printQuaternion(quaternion.normalized()); 
    customAssert((quaternion.normalized().norm() - 1.0f) < FLT_EPSILON, "normalizzazione");
    std::cout << std::endl;

    std::cout << "coniugato esterno: ";
    printQuaternion(quaternion.conjugated());
    customAssert(quaternion.conjugated() == conjugate, "coniugato esterno");
    std::cout << std::endl;

    std::cout << "coniugato interno: ";
    Graphics::Quaternion quat = quaternion;
    quat.conjugate();
    
    customAssert(quat == conjugate, "coniugato interno");
    std::cout << std::endl;

    std::cout << "verifica norma quadra = quat * coniugato: ";
    Graphics::Quaternion quatnorm = quaternion * quaternion.conjugated();
    printQuaternion(quatnorm);
    customAssert(quaternion.squaredNorm() == quatnorm.getRe(), "norma quadra = quat * coniugato");
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

    Graphics::Quaternion sumconj = quaternion + quaternion.conjugated();
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

    Graphics::Quaternion diffconj = quaternion - quaternion.conjugated();
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
    printQuaternionAxisAndAngle(quaternionRotator);
    Graphics::Vector3D rotated = quaternionRotator.rotate(vec);
    std::cout << "rotazione risultante: ";
    printVector3D(rotated);
    std::cout << std::endl;
    std::cout << std::endl;
}

void TestVectorClass()
{
    Graphics::Vector3D vec1 = Graphics::Vector3D::RIGHT;
    Graphics::Vector3D vec2 = Graphics::Vector3D::UP;

    Graphics::Vector3D sumtest(1, 1, 0);
    Graphics::Scalar dottest = 0;
    Graphics::Vector3D crosstest(0, 0, 1);

    customAssert((vec1 + vec2) == sumtest, "somma vettori");
    customAssert((dot(vec1, vec2) - dottest) <= FLT_EPSILON, "prodotto dot");
    customAssert(cross(vec1, vec2) == crosstest, "prodotto cross");
    customAssert(dot(vec1, vec1) == vec1.squaredNorm(), "prodotto dot = norma quadra");
    Graphics::Vector3D versor = (vec1 + vec2).normalize();
    printVector3D(vec1 + vec2);
    customAssert((versor.norm() - 1) <= FLT_EPSILON, "normalizzazione vettore");
    printVector3D(versor);
}
