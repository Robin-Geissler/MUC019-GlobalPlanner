
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

    EXPECT_NEAR(rayTracer.getRays().front().getDir().getX(), 0.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays().front().getDir().getY(),1.0, 0.001);

}

TEST(RayTracer_InitTest, correctInitVectors_2){
    RayTracer rayTracer = RayTracer(2);

    EXPECT_NEAR(rayTracer.getRays()[0].getDir().getX(), 1.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[0].getDir().getY(),1.73205, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[1].getDir().getX(), -1.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[1].getDir().getY(),1.73205, 0.001);
}

TEST(RayTracer_InitTest, correctInitVectors_5){
    RayTracer rayTracer = RayTracer(5);

    EXPECT_NEAR(rayTracer.getRays()[0].getDir().getX(), 1.73205, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[0].getDir().getY(),1.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[1].getDir().getX(), 1.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[1].getDir().getY(),1.73205, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[2].getDir().getX(), 0.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[2].getDir().getY(),1.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[3].getDir().getX(), -1.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[3].getDir().getY(),1.73205, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[4].getDir().getX(), -1.73205, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[4].getDir().getY(),1.0, 0.001);
}



int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}