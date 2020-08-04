#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#include <ifopt/ipopt_solver.h>

#include "model.h"
#include "formulations.h"

int main() {
    NlpFormulation formulation = turn();
    auto &kinematic_model = formulation.model_.kinematic_model_;

    ifopt::Problem nlp;
    SplineHolder solution;
    for (auto &c : formulation.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for (auto &c : formulation.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for (auto &c:formulation.GetCosts())
        nlp.AddCostSet(c);

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("max_cpu_time", 20.0);
    solver->Solve(nlp);

    std::fstream fs("output.csv", std::ios::out);
    const Eigen::IOFormat csv_format(Eigen::FullPrecision, 0, ",", "\n");

    std::vector<double> output;
    double t = 0.0;
    int num_steps = 0;
    while (t <= solution.base_linear_->GetTotalTime() + 1e-5) {
        output.push_back(t);

        Eigen::Vector3d base_lin = solution.base_linear_->GetPoint(t).p();
        output.insert(output.end(), {base_lin.x(), base_lin.y(), base_lin.z()});

        Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
        // Eigen::Vector3d deg = rad / M_PI * 180;
        output.insert(output.end(), {rad.x(), rad.y(), rad.z()});

        for (int i = 0; i < kinematic_model->GetNumberOfEndeffectors(); ++i) {
            Eigen::VectorXd ee(6);
            ee << solution.ee_motion_.at(i)->GetPoint(t).p(), solution.ee_force_.at(i)->GetPoint(t).p();
            std::vector<double> vec(ee.data(), ee.data() + ee.size());
            output.insert(output.end(), vec.begin(), vec.end());
        }

        t += 0.1;
        ++num_steps;
    }

    fs << "t,";
    fs << "base_x,base_y,base_z,base_roll,base_pitch,base_yaw,";\
    for (int i = 0; i < kinematic_model->GetNumberOfEndeffectors(); ++i) {
        char str[100];
        sprintf(str, "ee_%d_x,ee_%d_y,ee_%d_z,ee_%d_fx,ee_%d_fy,ee_%d_fz,", i, i, i, i, i, i);
        fs << str;
    }
    fs << "\n";

    Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> vec(output.data(), num_steps, output.size() / num_steps);
    fs << vec.format(csv_format) << "\n";

    return 0;
}
