#include "colour_matching.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <vector>

using namespace cv;

int main(int argc, char** argv)
{
    unsigned char values[] = { 0, 1, 1, 0, 0, 0, 1, 1 };

    std::vector<unsigned char> test(values, values+sizeof(values));			

    for(int i = 0; i < test.size(); i++)
    {
        std::cout<<(unsigned int)test[i]<<", ";
    }

    std::cout<<std::endl;

    std::vector<unsigned char> dilated = dilateLine(test);

    for(int i = 0; i < dilated.size(); i++)
    {
        std::cout<<(unsigned int)dilated[i]<<", ";
    }

    std::cout<<std::endl;

    std::vector<unsigned char> eroded = erodeLine(dilated);

    for(int i = 0; i < eroded.size(); i++)
    {
        std::cout<<(unsigned int)eroded[i]<<", ";	
    }

    std::cout<<std::endl;	

    unsigned int centroid = findBestCentroid(eroded);

    std::cout<<"Best Centroid: "<<centroid<<std::endl;


/*
    Mat source = imread("/home/turtlebot/coloured_squares.png");

    imshow("source", source);
    waitKey();

    Mat redOnly = redFilter(source);

    imshow("Red Only", redOnly);
    waitKey();

    Mat blueOnly = blueFilter(source);

    imshow("Blue Only", blueOnly);
    waitKey();

    Mat greenOnly = greenFilter(source);

    imshow("Green Only", greenOnly);
    waitKey();
*/
	
    return 0;

}

