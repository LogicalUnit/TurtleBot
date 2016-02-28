#ifndef COLOUR_MATCHING_H
#define COLOUR_MATCHING_H

#include <opencv2/core/core.hpp>

//Returns a black-and-white image where filtered colour is white ie 255
cv::Mat redFilter(const cv::Mat& src);
cv::Mat blueFilter(const cv::Mat& src);
cv::Mat greenFilter(const cv::Mat& src);

//Converts coloured image to grayscale
cv::Mat grayFilter(const cv::Mat& src);

//Returns a binary row of pixels
std::vector<unsigned char> getLine(const cv::Mat& img, unsigned int line); //for use with filtered images
std::vector<unsigned char> getGrayscaleLine(const cv::Mat& img, unsigned int line); //for use with grayscale images

//Used for finding blobs, binary in, binary out
std::vector<unsigned char> dilateLine(const std::vector<unsigned char>& row);
std::vector<unsigned char> erodeLine(const std::vector<unsigned char>& row);

//For use with binary row of pixels. Passes is number of time to dilate/erode
unsigned int findBestCentroid(const std::vector<unsigned char>& row, unsigned int passes = 5);



#endif
