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

#pragma once

#include <cmath>
#include <cstdio>
#include <iostream>

#include <Eigen/Dense>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

// Read a Bundle Adjustment in the Large dataset.
class BundleAdjuster {
public:
    ~BundleAdjuster() {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }

    int num_observations()       const { return num_observations_;               }
    const double* observations() const { return observations_;                   }
    double* mutable_cameras()          { return parameters_;                     }
    double* mutable_points()           { return parameters_  + 6 * num_cameras_; }
    double* mutable_camera_parameters(){ return parameters_  + 6 * num_cameras_ + 3 * num_points_; }

    double* mutable_camera_for_observation(int i) {
        return mutable_cameras() + camera_index_[i] * 6;
    }
    double* mutable_point_for_observation(int i) {
        return mutable_points() + point_index_[i] * 3;
    }

    bool LoadFile(const char* filename);
    void addFrame(int frame, Eigen::Vector3d camera_position, Eigen::AngleAxisd camera_orientation);
    void run(ceres::Solver::Summary* summary);

    Eigen::Vector3d getPosition(int frame);
    Eigen::AngleAxisd getOrientation(int frame);

private:
    template<typename T>
    void FscanfOrDie(FILE *fptr, const char *format, T *value) {
        int num_scanned = fscanf(fptr, format, value);
        if (num_scanned != 1) {
            std::cout << "Invalid UW data file." << std::endl;
        }
    }

    int num_cameras_;
    int num_points_;
    int num_observations_;
    int num_parameters_;
    int first_points_distance_;

    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* parameters_;

    ceres::Problem problem;

public:
// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
    struct SnavelyReprojectionError {
        SnavelyReprojectionError(double observed_x, double observed_y)
                : observed_x(observed_x), observed_y(observed_y) {}

        template <typename T>
        bool operator()(const T* const camera,
                        const T* const point,
                        const T* const camera_parameters,
                        T* residuals) const {
            // camera[0,1,2] are the angle-axis rotation.
            T p[3];
            ceres::AngleAxisRotatePoint(camera, point, p);

            // camera[3,4,5] are the translation.
            p[0] += camera[3];
            p[1] += camera[4];
            p[2] += camera[5];

            // Compute the center of distortion. The sign change comes from
            // the camera model that Noah Snavely's Bundler assumes, whereby
            // the camera coordinate system has a negative z axis.
            T xp = - p[0] / p[2];
            T yp = - p[1] / p[2];

            // Apply second and fourth order radial distortion.
            const T& l1 = camera_parameters[1];
            const T& l2 = camera_parameters[2];
            T r2 = xp*xp + yp*yp;
            T distortion = 1.0 + r2  * (l1 + l2  * r2);

            // Compute final projected point position.
            const T& focal = camera_parameters[0];
            T predicted_x = focal * distortion * xp;
            T predicted_y = focal * distortion * yp;

            // The error is the difference between the predicted and observed position.
            residuals[0] = predicted_x - observed_x;
            residuals[1] = predicted_y - observed_y;

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const double observed_x,
                                           const double observed_y) {
            return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3, 3>(
                    new SnavelyReprojectionError(observed_x, observed_y)));
        }

        double observed_x;
        double observed_y;
    };

    struct FirstPointDistanceError {
        FirstPointDistanceError(double desired_distance)
                : desired_distance(desired_distance) {}

        template <typename T>
        bool operator()(const T* const point1,
                        const T* const point2,
                        T* residuals) const {

            residuals[0] = (point1[0] - point2[0]) * (point1[0] - point2[0])
                         + (point1[1] - point2[1]) * (point1[1] - point2[1])
                         + (point1[2] - point2[2]) * (point1[2] - point2[2]);

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const double desired_distance) {
            return (new ceres::AutoDiffCostFunction<FirstPointDistanceError, 1, 3, 3>(
                    new FirstPointDistanceError(desired_distance)));
        }

        double desired_distance;
    };

};

