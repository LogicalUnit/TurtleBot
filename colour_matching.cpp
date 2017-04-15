#include "colour_matching.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <iostream>

cv::Mat redFilter(const cv::Mat& src)
{
    assert(src.type() == CV_8UC3);

    cv::Mat hsvSource;
    cv::cvtColor(src, hsvSource, CV_BGR2HSV);

    cv::Mat redOnly;
    cv::inRange(hsvSource, cv::Scalar(160, 0, 0), cv::Scalar(179, 255, 255), redOnly);
//  cv::inRange(hsvSource, cv::Scalar(0, 0, 0), cv::Scalar(19, 255, 255), redOnly);
//  cv::inRange(src, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 255), redOnly);
	
    return redOnly;
}

cv::Mat blueFilter(const cv::Mat& src)
{
    assert(src.type() == CV_8UC3);

    cv::Mat hsvSource;
    cv::cvtColor(src, hsvSource, CV_BGR2HSV);

    cv::Mat blueOnly;
    cv::inRange(hsvSource, cv::Scalar(75, 0, 0), cv::Scalar(130, 255, 255), blueOnly);

    return blueOnly;
}

cv::Mat greenFilter(const cv::Mat& src)
{
    assert(src.type() == CV_8UC3);

    cv::Mat hsvSource;
    cv::cvtColor(src, hsvSource, CV_BGR2HSV);

    cv::Mat greenOnly;
    cv::inRange(hsvSource, cv::Scalar(38, 0, 0), cv::Scalar(75, 255, 255), greenOnly);

    return greenOnly;
}

cv::Mat grayFilter(const cv::Mat& src)
{
    assert(src.type() == CV_8UC3);

    cv::Mat grayScale;
    cv::cvtColor(src, grayScale, CV_BGR2GRAY);

    return grayScale;	
}

//A slightly crappy way of estimating colour

/*
std::vector<int> scanFilteredLine(const cv::Mat& src, unsigned int scanline, unsigned int segment_size)
{
    int number_segments = (src.cols / segment_size) + 1; //number of segments to scan

    std::vector<int> segment_estimates;
    segment_estimates.resize(number_segments);

    for(int i = 0; i < number_segments; i++)
    {
        int black_pixels = 0;
        int white_pixels = 0;

        for (int j = i * segment_size; (j < (i+1) * segment_size) && (j < src.cols); j++) //scan each pixel in the segment
        {
            unsigned int pixel_colour = src.at<unsigned char>(scanline, j);	

            if (pixel_colour > 0)
                    white_pixels++;
            else
                    black_pixels++;
        }

        if (white_pixels > black_pixels)
            segment_estimates[i]= 1;
        else
            segment_estimates[i] = 0;	
    }

    return segment_estimates;
}
*/

std::vector<unsigned char> getLine(const cv::Mat& img, unsigned int line)
{
    assert(line < img.rows);

    std::vector<unsigned char> pixels;
    pixels.resize(img.cols);

    for (unsigned int i = 0; i < img.cols; i++)
    {
        unsigned char c = img.at<unsigned char>(line, i);

        if (c == 0)
        {
            pixels[i] = 0;
        }
        else
        {
            pixels[i] = 1;
        }
    }

    return pixels;
}

std::vector<unsigned char> getGrayscaleLine(const cv::Mat& img, unsigned int line)
{
    assert(line < img.rows);

    std::vector<unsigned char> pixels;
    pixels.resize(img.cols);

    for (unsigned int i = 0; i < img.cols; i++)
    {
        unsigned char c = img.at<unsigned char>(line, i);

        if (c < 190)
        {
            pixels[i] = 0;
        }
        else
        {
            pixels[i] = 1;
        }
    }

    return pixels;

}

std::vector<unsigned char> dilateLine(const std::vector<unsigned char>& row)
{
    std::vector<unsigned char> result(row);

    for (int i = 0; i < row.size(); i++)
    {
        if(i == 0) //first pixel
        {
            if (row[i+1] > 0)
            {
                result[i] = 1;
            }			 
        }
        else if (i == row.size() - 1) //last pixel
        {
            if(row[i-1] > 0)
            {
                result[i] = 1;
            }
        }
        else //all remaining pixels
        {
            if (row[i-1] > 0 || row[i+1] > 0)
            {
                result[i] = 1;
            }
        }
    }

    return result;
}

std::vector<unsigned char> erodeLine(const std::vector<unsigned char>& row)
{
    std::vector<unsigned char> result(row);

    for(int i = 0; i < row.size(); i++)
    {
        if (i == 0) //first pixel
        {
            if(row[i+1] == 0)
            {
                result[i] = 0;
            }			
        }
        else if (i == row.size() - 1) //last pixel
        {
            if(row[i-1] == 0)
            {
                result[i] = 0;
            }		
        }
        else //all remaining pixels
        {
            if (row[i-1] == 0 || row[i+1] == 0)
            {
                result[i] = 0;
            }									
        }
    }

    return result;
}

unsigned int blobCentre(const std::vector<unsigned char>& pixels)
{
    unsigned int lower_bound = 0, upper_bound = 0, best_lower = 0, best_upper = 0;

    for(int i = 0; i < pixels.size(); i++)
    {
        if (pixels[i] == 1)
        {
            upper_bound = i;

            if (upper_bound - lower_bound > best_upper - best_lower)
            {
                best_upper = upper_bound;
                best_lower = lower_bound;
            }		
        }
        else if (i < pixels.size()-1 && pixels[i+1] == 1) //zero followed by a one, ensuring we don't go past the end of the array
        {
            lower_bound = i+1;
            upper_bound = i+1;
        }
    //	std::cout<<"lower_bound: "<<lower_bound<<" upper_bound: "<<upper_bound<<std::endl;	
    }

    //  std::cout<<"best_lower: "<<best_lower<<" best upper: "<<best_upper<<std::endl;
    return (best_upper + best_lower) / 2;

}

unsigned int findBestCentroid(const std::vector<unsigned char>& row, unsigned int passes)
{
    std::vector<unsigned char> pixels(row);

    for (int i = 0; i < passes; i++)
    {
        pixels = dilateLine(pixels);	
    }

    for (int i = 0; i < passes; i++)
    {
        pixels = erodeLine(pixels);
    }

    unsigned int result = blobCentre(pixels);

    return result;	
}
