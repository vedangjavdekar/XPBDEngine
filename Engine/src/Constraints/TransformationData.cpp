#include "TransformationData.h"
#include "Constraints/PositionalConstraint.h"

namespace Simulation
{
    TransformationData GetTransformationData(Entity *e1, Entity *e2)
    {
        using namespace Eigen;
        TransformationData data;

        data.Entity1 = e1;
        data.Entity2 = e2;

        const Matrix3f E1RotationMat = e1->Rotation.toRotationMatrix();
        const Matrix3f E1RotationMatT = E1RotationMat.transpose();
        const Matrix3f E2RotationMat = e2->Rotation.toRotationMatrix();
        const Matrix3f E2RotationMatT = E2RotationMat.transpose();

        data.Entity1InvTensor = E1RotationMat * e1->InverseInertiaTensor * E1RotationMatT;
        data.Entity2InvTensor = E2RotationMat * e2->InverseInertiaTensor * E2RotationMatT;

        return data;
    }

    void ComputePositionalData(TransformationData &data, const Eigen::Vector3f& localR1, const Eigen::Vector3f& localR2)
    {
        data.WorldR1 = data.Entity1->Rotation.toRotationMatrix() * localR1;
        data.WorldR2 = data.Entity2->Rotation.toRotationMatrix() * localR2;
    }
}
