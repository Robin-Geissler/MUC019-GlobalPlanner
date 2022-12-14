#include "rayTracer.hpp"


/********************************************************
 *  Class Coordinate
 ********************************************************/

Coordinate::Coordinate(int x, int y) : x(x), y(y) {}

int Coordinate::getX() const {
    return x;
}

void Coordinate::setX(int x) {
    Coordinate::x = x;
}

int Coordinate::getY() const {
    return y;
}

void Coordinate::setY(int y) {
    Coordinate::y = y;
}

Coordinate Coordinate::operator+(const Coordinate &coo) {
    return Coordinate(x + coo.x, y + coo.y);
}

Coordinate Coordinate::operator-(const Coordinate &coo) {
    return Coordinate(x - coo.x, y - coo.y);
}

Coordinate Coordinate::operator*(const float &factor) {
    return Coordinate(x * factor, y * factor);
}

Coordinate Coordinate::operator/(const float &divisor) {
    return Coordinate(x / divisor, y / divisor);
}


/********************************************************
 *  Class Vec2
 ********************************************************/

Vec2::Vec2(float x, float y) : x(x), y(y) {}

Vec2::~Vec2() = default;

float Vec2::getX() const {
    return x;
}

void Vec2::setX(float x) {
    Vec2::x = x;
}

float Vec2::getY() const {
    return y;
}

void Vec2::setY(float y) {
    Vec2::y = y;
}

Vec2 Vec2::operator+(const Vec2& vec) {
    return Vec2(x+vec.x,y+vec.y);
}

Vec2 Vec2::operator-(const Vec2& vec) {
    return Vec2(x-vec.x, y-vec.y);
}

Vec2 Vec2::operator*(const float& factor) {
    return Vec2(x*factor, y*factor);
}


/********************************************************
 *  Class Ray
 ********************************************************/


bool Ray::outOfBounds(Coordinate field, const nav_msgs::OccupancyGrid& grid) {
    // check bounds (right, top, left, bottom)
    return field.getX() >= grid.info.width || field.getY() >= grid.info.height || field.getX() < 0 || field.getY() < 0;
}

bool Ray::edge(Coordinate field, const nav_msgs::OccupancyGrid& grid) {
    // check edge field (right, top, left)
    return field.getX() == grid.info.width - 1 || field.getY() == grid.info.height - 1 || field.getX() == 0 ;
}

bool Ray::occupied(Coordinate field, const nav_msgs::OccupancyGrid& grid) {
    // TODO set RAY_CRITICAL_COLLISION_PROBABILLITY in rayTracer.hpp
    return grid.data.at(getCoodinateIndex(field,grid)) > RAY_CRITICAL_COLLISION_PROBABILITY;
}



int Ray::decAbs(float *num) {
    if(*num > 0){
        (*num)--;
        return 1;
    } else{
        (*num)++;
        return -1;
    }
}

Ray::Ray(const Coordinate &start, const Vec2 &dir) : start(start), dir(dir) {
   }

Ray::~Ray() = default;

const Coordinate &Ray::getStart() const {
    return start;
}

void Ray::setStart(const Coordinate &start) {
    Ray::start = start;
}

const Vec2 &Ray::getDir() const {
    return dir;
}

void Ray::setDir(const Vec2 &dir) {
    Ray::dir = dir;
}


void Ray::setOccuGridFields(const nav_msgs::OccupancyGrid &inputGrid) {
    int curX = Ray::getStart().getX();  // current x coordinate
    int curY = Ray::getStart().getY();  // current y coordinate
    Coordinate newPoint(curX,curY);     // new Ray Point

    bool xIsMainDir = abs(Ray::getDir().getX()) >= abs(Ray::getDir().getY()); // tells if x is bigger than y
    float NextStepsX;   // tells how many steps to go in x direction before reload
    float NextStepsY;   // tells how many steps to go in y direction before reload

    // init NextSteps
    // only half way for the main direction in first init if half way >= 1
    // this will be full way after a refill
    if(xIsMainDir){
        NextStepsX = Ray::getDir().getX();
        NextStepsY = Ray::getDir().getY();
        if(NextStepsX >= 2){
            NextStepsX /= 2;
        }
    }else{
        NextStepsX = Ray::getDir().getX();
        NextStepsY = Ray::getDir().getY();
        if(NextStepsY >= 2){
            NextStepsY /= 2;
        }
    }

    // end the Loop if the Ray Collides with something
    while(!outOfBounds(newPoint,inputGrid) && !occupied(newPoint,inputGrid)){
        // add new Ray Point
        Ray::occuGridFields.push_back(newPoint);

        // find next Ray Point
        if(xIsMainDir){
            // go to next point, adjust NextSteps and cur accordingly
            getNextGridPoint(&NextStepsX, &NextStepsY, &curX, &curY);
            newPoint =  {curX, curY};
        } else{
            getNextGridPoint(&NextStepsY, &NextStepsX, &curY, &curX);
            newPoint = {curX,curY};
        }

        // if nextSteps in x and y direction is empty, refill it here
        if(abs(NextStepsX) < 1 && abs(NextStepsY) < 1){
            NextStepsX +=Ray::getDir().getX();
            NextStepsY +=Ray::getDir().getY();
        }
    }
}

void Ray::getNextGridPoint(float *mainDirNextSteps, float *subDirNextSteps, int *mainDirCoordinate, int *subDirCoordinate) {
    // go in main dir until it's empty
    if(abs(*mainDirNextSteps) >= 1){
        // go 1 step in x dir and dec mainDirNextSteps by 1
        *mainDirCoordinate += decAbs(mainDirNextSteps);
    }
    // go in sub dir
    else{
        // go 1 step in sub dir and dec subDirNextSteps by 1
        *subDirCoordinate += decAbs(subDirNextSteps);
    }
}

float Ray::getLength(const nav_msgs::OccupancyGrid& inputGrid) {
    // case: as long as possible - set to max_length = distance from Start to most outer corner of Grid
    if(edge(occuGridFields.back(), inputGrid)){
        return std::sqrt(((inputGrid.info.width - 1) / 2) * ((inputGrid.info.width - 1) / 2) + ((inputGrid.info.height - 1) * (inputGrid.info.height - 1)));
    }

    float horizontalLength = occuGridFields.back().getX() - start.getX();
    float verticalLength = occuGridFields.back().getY() - start.getY();

    return sqrtf(horizontalLength*horizontalLength + verticalLength*verticalLength);
}

Coordinate Ray::getCenter() {
    return (occuGridFields.back() + start) / 2;
}

int Ray::getCoodinateIndex(Coordinate field, const nav_msgs::OccupancyGrid& grid) {
    // calculate index
    int index = (field.getY() * grid.info.width) + field.getX();

    // asked field is out of Bounds
    if(index > grid.data.size()){
        throw std::range_error("field in Ray::getCoordinateIndex() is out of Bounds");
    }

    return index;
}







/********************************************************
 *  Class RayTracer
 ********************************************************/

RayTracer::RayTracer() = default;

RayTracer::RayTracer(int numberOfRays, int gridWidth, int gridHeight){

    // init inputGrid
    inputGrid.info.width = gridWidth;
    inputGrid.info.height = gridHeight;
    // init inputGrid data to 0
    for(int i = 0; i < gridWidth * gridHeight; i++){
        inputGrid.data.push_back(0);
    }

    // init outputGrid
    outputGrid.info.width = gridWidth;
    outputGrid.info.height = gridHeight;
    // init outputGrid data to 0
    for(int i = 0; i < gridWidth * gridHeight; i++){
        outputGrid.data.push_back(0);
    }

    // exception: numberOfRays is to low
    if(numberOfRays < 1){
        throw std::range_error("numberOfRays in RayTracer::Raytracer() is out of Range");
    }

    // init rays
    std::vector<Ray> rayVec;
    for(int i = 1; i <= numberOfRays; i++){
        /*
         * Create 1 Ray per loop cycle
         */
        // all rays should have the same angle to each other, so coordinates from the unit circle will be used
        float angle = ((float)i/(float)(numberOfRays + 1) * M_PIf32);
        Vec2 dir(cos(angle),sin(angle));
        // scale dir to set lower coordinate to +1 or -1
        // except if one coordinate is 0
        if(abs(dir.getX()) >= 0.0001 && abs(dir.getY() >= 0.0001)) {
            if (abs(dir.getX()) < abs(dir.getY())) {
                dir = dir * (1 / abs(dir.getX()));
            } else {
                dir = dir * (1 / abs(dir.getY()));
            }
        }
        // start is at bottom middle position
        Ray ray(Coordinate((gridWidth - 1) / 2, 0),dir);
        rayVec.push_back(ray);
    }
    RayTracer::rays = rayVec;

}

const std::vector<Ray> &RayTracer::getRays() const {
    return rays;
}

Ray RayTracer::getBestRay() {
    float rayBoost = 0;
    Ray bestRay = rays.front();
    float longestLength = bestRay.getLength(inputGrid);
    float currentLength;


    for(int i = 1; i < rays.size(); i++){
        // adjust Rayboost to strengthen inner Rays more than outer Rays
        if(i <= rays.size() / 2){
            rayBoost += RAY_BOOST_SCALOR;
        } else{
            rayBoost -= RAY_BOOST_SCALOR;
        }

        // if currentLength with boost is longer than longestLength with boost: select current Ray
        currentLength = rays[i].getLength(inputGrid) + rayBoost;
        if(currentLength > longestLength){
            bestRay = rays[i];
            longestLength = currentLength;
        }
    }
    return bestRay;
}

const nav_msgs::OccupancyGrid &RayTracer::getInputGrid() const {
    return inputGrid;
}

void RayTracer::setInputGrid(const nav_msgs::OccupancyGrid &inputGrid) {
    // update inputGrid
    RayTracer::inputGrid = inputGrid;
}

const nav_msgs::OccupancyGrid &RayTracer::getOutputGrid() const {
    return outputGrid;
}

void RayTracer::setOutputGrid() {
    // update all Rays
    for(auto & ray : rays){
        ray.setOccuGridFields(inputGrid);
    }

    // set all Points in outputGrid to 0
    for(int i = 0; i < outputGrid.data.size(); i++){
        outputGrid.data.data()[i] = 0;
    }

    // set the new output Point to 100
    outputGrid.data.data()[Ray::getCoodinateIndex(getBestRay().getCenter(),outputGrid)] = 50;
}




