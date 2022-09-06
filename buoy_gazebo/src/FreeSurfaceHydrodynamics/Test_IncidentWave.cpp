// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "LinearIncidentWave.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <tuple>

#include <Eigen/Dense>

using namespace Eigen;

#include <gnuplot-iostream.h>

int main()
{
  LinearIncidentWave Inc;

  //Inc.SetToPiersonMoskowitzSpectrum(1, 0, 300);
  //Inc.SetToPiersonMoskowitzSpectrum(6, 0);
  Inc.SetToMonoChromatic(1, 12, 90*M_PI/180);

  std::cout << Inc << std::endl;

  std::vector<double> pts_tau, pts_eta, pts_etadot, pts_cos;

  for(double t = 0; t<600; t+=0.1)
  {
    std::cout << t
        << "  " << Inc.eta(0,0,t)
        << "  " << Inc.etadot(0,0,t)
        << "  " << cos(2*M_PI*t/12)
        << std::endl;

    pts_tau.push_back(t);
    pts_eta.push_back(Inc.eta(0,0,t));
    pts_etadot.push_back(Inc.etadot(0,0,t));
    pts_cos.push_back(cos(2*M_PI*t/12));
  }

  // plotting
  Gnuplot gp;

  gp << "set term qt title 'Test Incident Wave'\n";
  gp << "set multiplot layout 2,1 rowsfirst \n";
  gp << "set grid\n";
  gp << "plot "
      << "'-' u 1:2 with lines title 'eta',"
      << "'-' u 1:2 with lines title 'etadot'\n";
  gp.send1d(std::make_tuple(pts_tau, pts_eta));
  gp.send1d(std::make_tuple(pts_tau, pts_etadot));
  gp << "set xlabel 'sec'\n";
  gp << "set ylabel 'm'\n";

  return 0;
}

