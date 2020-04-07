
#include "gtest/gtest.h"
#include "rayTracer.hpp"


TEST(RayTracer_InitTest, noError){
    EXPECT_NO_THROW(RayTracer rayTracer = RayTracer(1););
}

TEST(RayTracer_InitTest, correctInitVectorNumber_1){
    RayTracer rayTracer = RayTracer(1);
    EXPECT_EQ(rayTracer.getRays().size(), 1);
}

TEST(RayTracer_InitTest, correctInitVectorNumber_4){
    RayTracer rayTracer = RayTracer(4);
    EXPECT_EQ(rayTracer.getRays().size(), 4);
}

TEST(RayTracer_InitTest, correctInitVectors_1){
    RayTracer rayTracer = RayTracer(1);

    EXPECT_FLOAT_EQ(rayTracer.getRays().front().getDir().getX(), 0.0);
    EXPECT_FLOAT_EQ(rayTracer.getRays().front().getDir().getY(),1.0);
}



int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}