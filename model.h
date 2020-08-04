//
// Created by cryscan on 8/3/20.
//

#ifndef QUADRUPED_MODEL_H
#define QUADRUPED_MODEL_H

#include <towr/models/endeffector_mappings.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/kinematic_model.h>

enum StalkerGait {
    GALLOP, TROT,
};

struct StalkerKinematicModel : public towr::KinematicModel {
    explicit StalkerKinematicModel(StalkerGait gait = TROT);
};

struct StalkerDynamicModel : public towr::SingleRigidBodyDynamics {
    StalkerDynamicModel();
};

#endif //QUADRUPED_MODEL_H
