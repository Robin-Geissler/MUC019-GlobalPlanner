
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

TEST(Ray_SetOccuGridFields, getNextGridPoint){
    Ray ray(Coordinate(1,2),Vec2(1,2));
    float mainDirNextS = 2.5;
    float subDirNexS = 1.0;
    int mainCoordinate = 4;
    int subCoordinate = 2;
    Coordinate testCo1 = ray.getNextGridPoint(&mainDirNextS, &subDirNexS, &mainCoordinate, &subCoordinate);
    EXPECT_EQ(mainDirNextS,1.5);
    EXPECT_EQ(subDirNexS,1.0);
    EXPECT_EQ(mainCoordinate,5);
    EXPECT_EQ(subCoordinate,2);

    Coordinate testCo2 = ray.getNextGridPoint(&mainDirNextS, &subDirNexS, &mainCoordinate, &subCoordinate);
    EXPECT_EQ(mainDirNextS,0.5);
    EXPECT_EQ(subDirNexS,1.0);
    EXPECT_EQ(mainCoordinate,6);
    EXPECT_EQ(subCoordinate,2);

    Coordinate testCo3 = ray.getNextGridPoint(&mainDirNextS, &subDirNexS, &mainCoordinate, &subCoordinate);
    EXPECT_EQ(mainDirNextS,0.5);
    EXPECT_EQ(subDirNexS,0.0);
    EXPECT_EQ(mainCoordinate,6);
    EXPECT_EQ(subCoordinate,3);

    EXPECT_EQ(testCo1.getX(),5);
    EXPECT_EQ(testCo1.getY(),2);
    EXPECT_EQ(testCo2.getX(),6);
    EXPECT_EQ(testCo2.getY(),2);
    EXPECT_EQ(testCo3.getX(),6);
    EXPECT_EQ(testCo3.getY(),3);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}