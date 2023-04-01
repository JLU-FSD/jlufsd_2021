#include "smoother_test.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "smoother_test");
  ros::NodeHandle nodeHandle("~");

  ros::Rate loop_rate(10);
  planning::Path path;
  planning::Path smoothed_path;
  readFromCsv(path, "/home/auto/fsd/src/planning/data/test_path.csv");
  
  
  // Rotate3orderSmoother smoother;
  IterativePolynomialSmoother resampler;
  // DenseResamplingSmoother resampler;
  path =resampler.process(path);
  path.calculateS();
  path.calculateYaw();
  path.calculateCur();
  path.showInCsv("/home/auto/fsd/src/planning/data/resampler.csv");

  // PurePursuitSmoother smoother;
  // IterativePolynomialSmoother smoother;
  // LqrSmoother smoother;
  // // StanleySmoother smoother;
  // smoothed_path = smoother.process(path);
  // smoothed_path.calculateS();
  // smoothed_path.calculateYaw();
  // smoothed_path.calculateCur();
  // smoothed_path.showInCsv("/home/auto/fsd/src/planning/data/test_reslut.csv");
  return 0;
}