#include "CameraModel.h"
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <random>

using namespace mcam;

TEST(CameraModelTest, RealCameraReprojection) {
    // some real-world data
    double scale = 604.0;
    Vector2 center(1.58492, 1.07424);
    int unmapPolyDeg = 5;
    VectorX unmapPolyCoeffs(unmapPolyDeg, 1);
    unmapPolyCoeffs << 1.14169, -0.203229, -0.362134, 0.351011, -0.147191;
    int width = 1920, height = 1208;
    CameraModel cam(width, height, scale, center, unmapPolyCoeffs);

    std::srand(42);
    const int testnum = 2000;
    double sqErr = 0.0;

    for (int i = 0; i < testnum; ++i) {
        Vector2 pnt(double(rand() % width), double(rand() % height));
        Vector3 ray = cam.unmap(pnt.data());
        ray *= 10.5;

        Vector2 pntBack = cam.map(ray.data());
        sqErr += (pnt - pntBack).squaredNorm();
    }

    double rmse = std::sqrt(sqErr / testnum); // rmse in pixels
    LOG(INFO) << "reprojection rmse = " << rmse << std::endl;
    EXPECT_LT(rmse, 0.1);
}

TEST(CameraModelTest, SolelyPolynomial) {
    // here camera is initialised so that its mapping only inverses the given
    // polynomial
    double scale = 1;
    Vector2 center(0.0, 0.0);
    int unmapPolyDeg = 2;
    VectorX unmapPolyCoefs(unmapPolyDeg, 1);
    unmapPolyCoefs << 1.0, -1.0;
    int width = 2, height = 0;
    CameraModel cam(width, height, scale, center, unmapPolyCoefs);

    std::srand(42);
    const int testnum = 2000;
    double sqErr = 0.0;
    for (int i = 0; i < testnum; ++i) {
        double x = double(std::rand()) / RAND_MAX;
        Vector3 pnt(x, 0, 1 - x * x);
        Vector2 projected = cam.map(pnt.data());
        sqErr += (projected - Vector2(x, 0)).squaredNorm();
    }
    double rmse = std::sqrt(sqErr / testnum);
    LOG(INFO) << "rmse = " << rmse << std::endl;
    EXPECT_LT(rmse, 0.0005);
}

TEST(CameraModelTest, CamerasPyramid) {
    double scale = 604.0;
    Vector2 center(1.58447, 1.07353);
    int unmapPolyDeg = 7;
    int pyrLevels = 6;
    VectorX unmapPolyCoeffs(unmapPolyDeg, 1);
    unmapPolyCoeffs << 1.14544, -0.146714, -0.967996, 2.13329, -2.42001, 1.33018,
            -0.292722;
    int width = 1920, height = 1208;
    CameraModel cam(width, height, scale, center, unmapPolyCoeffs);
    StdVectorA<CameraModel> camPyr = cam.camPyr(pyrLevels);

    std::mt19937 mt(42);
    std::uniform_real_distribution<> xs(0, width - 1);
    std::uniform_real_distribution<> ys(0, height - 1);

    const int testCount = 1000;
    for (int lvl = 0; lvl < pyrLevels; ++lvl)
        for (int it = 0; it < testCount; ++it) {
            double x = xs(mt), y = ys(mt);
            Vector2 pnt(x, y);
            Vector2 pntScaled(x / (1 << lvl), y / (1 << lvl));
            Vector3 unmapOrig = cam.unmap(pnt).normalized();
            Vector3 unmapPyr = camPyr[lvl].unmap(pntScaled).normalized();
            double cos = unmapOrig.dot(unmapPyr);
            if (cos < -1)
                cos = -1;
            if (cos > 1)
                cos = 1;
            double angle = (180.0 / M_PI) * std::acos(cos);
            EXPECT_LT(angle, 0.01);
        }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    return RUN_ALL_TESTS();
}
