// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

#include "BundleAdjuster.h"

bool BundleAdjuster::LoadFile(const char* filename) {
    return LoadFile(filename, true);
}

bool BundleAdjuster::LoadFile(const char* filename, bool do_scaling) {
    FILE* fptr = fopen(filename, "r");
    if (fptr == NULL) {
      return false;
    };

    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);
    if (do_scaling){
        FscanfOrDie(fptr, "%d", &first_points_distance_);
    }

    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    num_parameters_ = 6 * num_cameras_ + 3 * num_points_ + 3;
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2*i + j);
      }
    }

    if (do_scaling) {
        // distance of first two points
        ceres::CostFunction *cost_function =
                FirstPointDistanceError::Create(first_points_distance_);

        problem.AddResidualBlock(cost_function, NULL, mutable_points(), mutable_points() + 3);
    }

    return true;
}

void BundleAdjuster::addFrame(int frame, Eigen::Vector3d camera_position, Eigen::AngleAxisd camera_orientation){
    double *camera = mutable_cameras() + frame * 6;
    camera[3] = camera_position.x();
    camera[4] = camera_position.y();
    camera[5] = camera_position.z();

    camera[0] = camera_orientation.angle() * camera_orientation.axis()[0];
    camera[1] = camera_orientation.angle() * camera_orientation.axis()[1];
    camera[2] = camera_orientation.angle() * camera_orientation.axis()[2];

    for (int i = 0; i < num_observations(); ++i) {
        if (camera_index_[i] != frame) continue;

        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.

        ceres::CostFunction* cost_function =
                SnavelyReprojectionError::Create(observations()[2 * i + 0],
                                                 observations()[2 * i + 1]);
        problem.AddResidualBlock(cost_function,
                                 NULL /* squared loss */,
                                 mutable_camera_for_observation(i),
                                 mutable_point_for_observation(i),
                                 mutable_camera_parameters());
    }
}

void BundleAdjuster::run(ceres::Solver::Summary* summary){
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solve(options, &problem, summary);
}

Eigen::Vector3d BundleAdjuster::getPosition(int frame) {
    double *camera = mutable_cameras() + frame * 6;
    return Eigen::Vector3d(camera[3], camera[4], camera[5]);
}

Eigen::AngleAxisd BundleAdjuster::getOrientation(int frame) {
    double *camera = mutable_cameras() + frame * 6;
    Eigen::Vector3d raw_orientation(camera[0], camera[1], camera[2]);
    return Eigen::AngleAxisd(raw_orientation.norm(), raw_orientation.normalized());
}

/*int main(int argc, char** argv) {

  const double* observations = bal_problem.observations();

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.

    ceres::CostFunction* cost_function =
        SnavelyReprojectionError::Create(observations[2 * i + 0],
                                         observations[2 * i + 1]);
    problem.AddResidualBlock(cost_function,
                             NULL /* squared loss * /,
                             bal_problem.mutable_camera_for_observation(i),
                             bal_problem.mutable_point_for_observation(i));
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return 0;
}*/
