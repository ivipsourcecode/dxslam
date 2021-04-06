/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*
* DXSLAM，A Robust and Efficient Visual SLAM System with Deep Features，is based on the famous ORB-SLAM2. 
* Copyright (C) 2020, iVip Lab @ EE, THU (https://ivip-tsinghua.github.io/iViP-Homepage/). All rights reserved.
* Licensed under the GPLv3 License;
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* https://github.com/ivipsourcecode/dxslam/blob/master/License-gpl.txt
*/
#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include "MapPoint.h"
#include "Frame.h"

namespace DXSLAM
{

class PnPsolver {
 public:
  PnPsolver(const Frame &F, const std::vector<MapPoint*> &vpMapPointMatches);

  ~PnPsolver();

  void SetRansacParameters(double probability = 0.99, int minInliers = 8 , int maxIterations = 300, int minSet = 4, float epsilon = 0.4,
                           float th2 = 5.991);

  cv::Mat find(std::vector<bool> &vbInliers, int &nInliers);

  cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

 private:

  void CheckInliers();
  bool Refine();

  // Functions from the original EPnP code
  void set_maximum_number_of_correspondences(const int n);
  void reset_correspondences(void);
  void add_correspondence(const double X, const double Y, const double Z,
              const double u, const double v);

  double compute_pose(double R[3][3], double T[3]);

  void relative_error(double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  double reprojection_error(const double R[3][3], const double t[3]);

  void choose_control_points(void);
  void compute_barycentric_coordinates(void);
  void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
  void compute_ccs(const double * betas, const double * ut);
  void compute_pcs(void);

  void solve_for_sign(void);

  void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  double dot(const double * v1, const double * v2);
  double dist2(const double * p1, const double * p2);

  void compute_rho(double * rho);
  void compute_L_6x10(const double * ut, double * l_6x10);

  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
  void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
				    double cb[4], CvMat * A, CvMat * b);

  double compute_R_and_t(const double * ut, const double * betas,
			 double R[3][3], double t[3]);

  void estimate_R_and_t(double R[3][3], double t[3]);

  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  void mat_to_quat(const double R[3][3], double q[4]);


  double uc, vc, fu, fv;

  double * pws, * us, * alphas, * pcs;
  int maximum_number_of_correspondences;
  int number_of_correspondences;

  double cws[4][3], ccs[4][3];
  double cws_determinant;

  std::vector<MapPoint*> mvpMapPointMatches;

  // 2D Points
  std::vector<cv::Point2f> mvP2D;
  std::vector<float> mvSigma2;

  // 3D Points
  std::vector<cv::Point3f> mvP3Dw;

  // Index in Frame
  std::vector<size_t> mvKeyPointIndices;

  // Current Estimation
  double mRi[3][3];
  double mti[3];
  cv::Mat mTcwi;
  std::vector<bool> mvbInliersi;
  int mnInliersi;

  // Current Ransac State
  int mnIterations;
  std::vector<bool> mvbBestInliers;
  int mnBestInliers;
  cv::Mat mBestTcw;

  // Refined
  cv::Mat mRefinedTcw;
  std::vector<bool> mvbRefinedInliers;
  int mnRefinedInliers;

  // Number of Correspondences
  int N;

  // Indices for random selection [0 .. N-1]
  std::vector<size_t> mvAllIndices;

  // RANSAC probability
  double mRansacProb;

  // RANSAC min inliers
  int mRansacMinInliers;

  // RANSAC max iterations
  int mRansacMaxIts;

  // RANSAC expected inliers/total ratio
  float mRansacEpsilon;

  // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
  float mRansacTh;

  // RANSAC Minimun Set used at each iteration
  int mRansacMinSet;

  // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
  std::vector<float> mvMaxError;

};

} //namespace ORB_SLAM

#endif //PNPSOLVER_H
