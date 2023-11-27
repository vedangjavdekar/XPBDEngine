#pragma once
#include <Eigen/Dense>
#include "Engine/Entity.h"

namespace Simulation
{
    struct TransformationData
    {
        Entity *Entity1 = nullptr;
        Entity *Entity2 = nullptr;

        Eigen::Vector3f WorldR1;
        Eigen::Vector3f WorldR2;

        Eigen::Matrix3f Entity1InvTensor;
        Eigen::Matrix3f Entity2InvTensor;
    };

    TransformationData GetTransformationData(Entity *e1, Entity *e2);
    void ComputePositionalData(TransformationData &data, const Eigen::Vector3f& localR1, const Eigen::Vector3f& localR2);
}
