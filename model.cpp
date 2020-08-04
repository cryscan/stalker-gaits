//
// Created by cryscan on 8/3/20.
//

#include "model.h"

using namespace towr;

StalkerKinematicModel::StalkerKinematicModel(StalkerGait gait) : KinematicModel(4) {
    const double x_nominal_b = 0.68;
    const double y_nominal_b = 0.38;
    const double z_nominal_b = -0.84;

    nominal_stance_.at(LF) << x_nominal_b, y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) << x_nominal_b, -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b, y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b, -y_nominal_b, z_nominal_b;

    if (gait == GALLOP) max_dev_from_nominal_ << 0.4, 0.1, 0.2;
    else max_dev_from_nominal_ << 0.5, 0.2, 0.1;
}

StalkerDynamicModel::StalkerDynamicModel() : SingleRigidBodyDynamics(
        80.0,
        10.26645, 21.09592, 21.89397, 0.010176, -0.645842, -0.015873,
        4
) {}