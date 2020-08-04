//
// Created by cryscan on 8/3/20.
//

#include <unsupported/Eigen/EulerAngles>

#include "model.h"
#include "formulations.h"

using namespace towr;

NlpFormulation trot_turn() {
    NlpFormulation formulation;

    formulation.terrain_ = std::make_shared<FlatGround>(0);

    formulation.model_.kinematic_model_ = std::make_shared<StalkerKinematicModel>();
    formulation.model_.dynamic_model_ = std::make_shared<StalkerDynamicModel>();

    double angle = -M_PI_2;
    Eigen::EulerAnglesXYZd euler(0.0, 0.0, angle);

    Eigen::Vector3d init_pos(0, 0, 0.8);
    Eigen::Vector3d init_ang(0, 0, angle);
    formulation.initial_base_.lin.at(kPos) = init_pos;
    formulation.initial_base_.ang.at(kPos) = init_ang;

    auto& kinematic_model = formulation.model_.kinematic_model_;
    for (int i = 0; i < kinematic_model->GetNumberOfEndeffectors(); ++i) {
        Eigen::Vector3d ee_pos = init_pos + kinematic_model->GetNominalStanceInBase().at(i);
        ee_pos.z() = 0;
        ee_pos = euler * ee_pos;
        formulation.initial_ee_W_.push_back(ee_pos);
    }

    formulation.final_base_.lin.at(kPos) << 2.0, 2.0, 0.8;

    formulation.params_.ee_in_contact_at_start_ = {true,
                                                   true,
                                                   true,
                                                   true};
    formulation.params_.ee_phase_durations_ = {{0.1, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 1.1},
                                               {1.1, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.1},
                                               {1.1, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.1},
                                               {0.1, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 1.1}};
    formulation.params_.OptimizePhaseDurations();

    return formulation;
}