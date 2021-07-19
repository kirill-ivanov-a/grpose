#include <glog/logging.h>
#include <gtest/gtest.h>

TEST(DemoTestSuite, DemoTestPass) { EXPECT_TRUE(true); }

// TEST(DemoTestSuite, DemoTestFail) {
//     EXPECT_TRUE(false) << "This test is expected to fail and will be removed in the future";
// }

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    return RUN_ALL_TESTS();
}
