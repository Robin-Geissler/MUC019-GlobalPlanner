
#include "gtest/gtest.h"
#include "rayTracer.hpp"


TEST(RayTracer_InitTest, noError){
    EXPECT_NO_THROW(RayTracer rayTracer = RayTracer(1,3,3););
}

TEST(RayTracer_InitTest, correctInitVectorNumber_1){
    RayTracer rayTracer = RayTracer(1,3,3);
    EXPECT_EQ(rayTracer.getRays().size(), 1);
}

TEST(RayTracer_InitTest, correctInitVectorNumber_4){
    RayTracer rayTracer = RayTracer(4,3,3);
    EXPECT_EQ(rayTracer.getRays().size(), 4);
}

TEST(RayTracer_InitTest, correctInitVectors_1){
    RayTracer rayTracer = RayTracer(1,3,3);

    EXPECT_NEAR(rayTracer.getRays().front().getDir().getX(), 0.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays().front().getDir().getY(),1.0, 0.001);

}

TEST(RayTracer_InitTest, correctInitVectors_2){
    RayTracer rayTracer = RayTracer(2,3,3);

    EXPECT_NEAR(rayTracer.getRays()[0].getDir().getX(), 1.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[0].getDir().getY(),1.73205, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[1].getDir().getX(), -1.0, 0.001);
    EXPECT_NEAR(rayTracer.getRays()[1].getDir().getY(),1.73205, 0.001);
}

TEST(RayTracer_InitTest, correctInitVectors_5){
    RayTracer rayTracer = RayTracer(5,3,3);

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
    ray.getNextGridPoint(&mainDirNextS, &subDirNexS, &mainCoordinate, &subCoordinate);
    EXPECT_EQ(mainDirNextS,1.5);
    EXPECT_EQ(subDirNexS,1.0);
    EXPECT_EQ(mainCoordinate,5);
    EXPECT_EQ(subCoordinate,2);

    ray.getNextGridPoint(&mainDirNextS, &subDirNexS, &mainCoordinate, &subCoordinate);
    EXPECT_EQ(mainDirNextS,0.5);
    EXPECT_EQ(subDirNexS,1.0);
    EXPECT_EQ(mainCoordinate,6);
    EXPECT_EQ(subCoordinate,2);

    ray.getNextGridPoint(&mainDirNextS, &subDirNexS, &mainCoordinate, &subCoordinate);
    EXPECT_EQ(mainDirNextS,0.5);
    EXPECT_EQ(subDirNexS,0.0);
    EXPECT_EQ(mainCoordinate,6);
    EXPECT_EQ(subCoordinate,3);

}

TEST(RayTracer_GridTest, TestGrid_1){
    RayTracer rayTracer = RayTracer(5,3,3);

    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 1.0;
    grid.info.width = 3;
    grid.info.height = 3;

    // load map
    for(int i = 0; i < 9; i++){
        grid.data.push_back(0);
    }

    rayTracer.setInputGrid(grid);
    Ray ray = rayTracer.getBestRay();

    EXPECT_NEAR(ray.getLength(grid),2.23607, 0.001);
    EXPECT_NEAR(ray.getDir().getX(), 0.0, 0.001);
    EXPECT_NEAR(ray.getDir().getY(),1.0, 0.001);
}

TEST(RayTracer_GridTest, TestGrid_2){
    RayTracer rayTracer = RayTracer(5,5,5);

    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = 1.0;
    grid.info.width = 5;
    grid.info.height = 5;

    // load map
    for(int i = 0; i < 25; i++){
        if(i >=17 && i <= 19){
            grid.data.push_back(100);
        } else{
            grid.data.push_back(0);
        }
    }

    rayTracer.setInputGrid(grid);
    Ray ray = rayTracer.getBestRay();
    rayTracer.setOutputGrid();

    EXPECT_NEAR(ray.getLength(grid),4.472135, 0.001);
    EXPECT_NEAR(ray.getDir().getX(), -1.0, 0.001);
    EXPECT_NEAR(ray.getDir().getY(),1.73285, 0.001);
    for(int i = 0; i < 25; i++){
        if(i == 11){
            EXPECT_EQ(rayTracer.getOutputGrid().data.at(i), 100);
        } else{
            EXPECT_EQ(rayTracer.getOutputGrid().data.at(i),0);
        }
    }
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}