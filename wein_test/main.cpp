//
// Created by wein on 7/20/18.
//

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <ctime>
#include <cstdio>

struct Transform {

    Eigen::Transform<double, 3, Eigen::Affine> m_transform;

    template<typename T>
    void setTranslation(const T& t) {}

    template<typename T>
    void setRotation(const T& t) {}

    template<typename T>
    void setScale(const T& t) {}

    Transform compute(const ::Transform& other, double t) {

        Eigen::Matrix3d scalingMatrix, rotationMatrix;
        m_transform.computeRotationScaling(&rotationMatrix, &scalingMatrix);

        Eigen::Quaterniond thisRot(rotationMatrix);
        Eigen::Vector3d thisScale(scalingMatrix(0), scalingMatrix(4), scalingMatrix(8));
        other.m_transform.computeRotationScaling(&rotationMatrix, &scalingMatrix);

        Eigen::Quaterniond otherRot(rotationMatrix);
        Eigen::Vector3d otherScale(scalingMatrix(0), scalingMatrix(4), scalingMatrix(8));

        Eigen::Vector3d transR = m_transform.translation() * (1.0 - t) + other.m_transform.translation() * t;
        Eigen::Quaterniond quatR = thisRot.slerp(t, otherRot);
        Eigen::Vector3d scaleR = thisScale * (1.0 -t) + otherScale * t;
        Transform result;
        result.setTranslation(transR);
        result.setRotation(quatR);
        result.setScale(scaleR);
        return result;
    }
};



int main() {
    Transform sut, other;
    double v{1.23};
    std::clock_t start = std::clock();
    for (int i=500; i--; ) {
        sut.compute(other, v);
    }
    printf("%0.3f us\n", (float)(std::clock() - start));

    return 0;
}