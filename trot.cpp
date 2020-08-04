//
// Created by cryscan on 8/3/20.
//

#include "model.h"
#include "formulations.h"

using namespace towr;

NlpFormulation trot() {
    NlpFormulation formulation;

    formulation.terrain_ = std::make_shared<FlatGround>(0);

    formulation.model_.kinematic_model_ = std::make_shared<StalkerKinematicModel>();
    formulation.model_.dynamic_model_ = std::make_shared<StalkerDynamicModel>();

    Eigen::Vector3d init_pos(0, 0, 0.8);
    formulation.initial_base_.lin.at(kPos) = init_pos;

    auto &kinematic_model = formulation.model_.kinematic_model_;
    for (int i = 0; i < kinematic_model->GetNumberOfEndeffectors(); ++i) {
        Eigen::Vector3d ee_pos = init_pos + kinematic_model->GetNominalStanceInBase().at(i);
        ee_pos.z() = 0;
        formulation.initial_ee_W_.push_back(ee_pos);
    }

    formulation.final_base_.lin.at(kPos) << 4.0, 0.0, 0.8;

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