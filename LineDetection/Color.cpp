#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <string>

using namespace cv;
using std::vector;
using std::string;
#define PI          3.14159265
#define STEPSIZE    10
#define SWAPDIST    80
#define OFFTRACK    500
#define SECTION     80
#define IMAGE_W     1280
#define IMAGE_H     720
#define AREA_LIMIT  20000
//#define LIGHT_FACTOR 0.1

struct line_bin
{
    int count = 0;
    Point upper;
    Point lower;
    bool empty = true;
};

void fix_lines(vector<Vec4i> lines ,int bin_size, line_bin *bin);
vector<Eigen::Vector2d> get_route(int bin_size, line_bin *bin);
Mat simple_threshold(Mat image, int thresholdGray);
Mat discrete_threshold(Mat image, int thresholdGray, int light);
void largest_contour(Mat binaryIm, Mat &contourDrawing, bool &bigArea);

Eigen::Vector2d convert(Point in)
{
    return Eigen::Vector2d(in.x, in.y);
}

int main( )
{
    string path = "testLine3.jpg";

    Mat image;

    //image = imread( path, CV_LOAD_IMAGE_GRAYSCALE );
    image = imread( path, CV_LOAD_IMAGE_COLOR );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);


    // Create a window
    namedWindow("slider", 1);

    int thresholdGray = 40;
    createTrackbar("gray", "slider", &thresholdGray, 255);

    int light = 20;
    createTrackbar("factor", "slider", &light, 255);

    Mat binaryIm;

    while (true)
    {
        //Mat binaryIm = simple_threshold(image, thresholdGray);
        Mat binaryIm = discrete_threshold(image, thresholdGray, light);

        imshow("slider", binaryIm);

        Mat drawing;
        bool verification;
        largest_contour(binaryIm, drawing, verification);

        //std::cout << verification << std::endl;

        if(verification == true)
        {
            /// Show in a window
            namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
            imshow( "Contours", drawing );

            //actual code that does something else than drawing
            vector<Vec4i> lines;
            HoughLinesP( drawing, lines, 1, CV_PI/180, 80, 30, 10 );
            //------------------------------------------------


            Mat lines_drawing = Mat::zeros( binaryIm.size(), CV_8UC3 );
            for( size_t i = 0; i < lines.size(); i++ )
            {
                line( lines_drawing, Point(lines[i][0], lines[i][1]),
                    Point(lines[i][2], lines[i][3]), Scalar(255,255,255), 3, 8 );
            }


            int bin_size = 19;
            line_bin bin[bin_size];
            fix_lines(lines, bin_size, bin);


            for(int i=0; i<bin_size; i++)
            {
                //use the endpoints for a new line
                if(bin[i].empty == false)
                {
                    line( lines_drawing, bin[i].upper, bin[i].lower, Scalar(255,0,0), 2, 8 );
                }
            }

            //Histogram of discretized lines
            //Mat histImage( 50, bin_size*10, CV_8UC1, Scalar( 0,0,0) );
            //for( int i = 1; i < bin_size; i++ )
            //{
            //    line( histImage, Point( 10*(i-1),  bin[i-1].count) ,
            //                     Point( 10*(i),    bin[i].count),
            //                     Scalar( 255, 255, 255), 2, 8, 0  );
            //}

            //imshow( "hist of angles", histImage );

            vector<Eigen::Vector2d> route = get_route(bin_size, bin);

            for(int i=0; i < route.size(); i++)
            {
                circle(lines_drawing, Point(route[i][0], route[i][1]), 3, Scalar(0,255,255),2);

            }

            namedWindow( "Detected Lines", CV_WINDOW_AUTOSIZE );
            imshow( "Detected Lines", lines_drawing );
        }

        // Wait until user press some key for 50ms
        int iKey = waitKey(50);

        //if user press 'ESC' key
        if (iKey == 27)
        {
            break;
        }
    }

    //imwrite("edges.png", edge);

    return 0;
}

void largest_contour(Mat binaryIm, Mat &contourDrawing, bool &bigArea)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( binaryIm, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    double largest_area=0;
    int contour_index=0;

    for( int i = 0; i< contours.size(); i++ )
    {
        double area = contourArea(contours[i]);
        if(area > largest_area)
        {
            largest_area = area;
            contour_index = i;
        }
    }

    //std::cout << "area: " << largest_area << std::endl;
    if(largest_area > AREA_LIMIT)
    {
        bigArea = true;
    }
    else
    {
        bigArea = false;
    }

    Scalar color = Scalar( 255, 255, 255 );
    Mat drawing = Mat::zeros( binaryIm.size(), CV_8UC1 );
    drawContours( drawing, contours, contour_index, color, 2, 8, hierarchy, 0, Point() );

    contourDrawing = drawing;
}

void fix_lines(vector<Vec4i> lines ,int bin_size, line_bin *bin)
{
    vector<double> angle;
    vector<double> x[bin_size];
    vector<double> y[bin_size];

    for(int i = 0; i < lines.size(); i++)
    {
        double theta = atan( ((double)(lines[i][1]-lines[i][3]))/((double)(lines[i][0]-lines[i][2])));
        angle.push_back(theta);
        //decide for a bin
        int bin_index = (int)((theta+PI/2)/PI * (bin_size-1) );

        //update its status
        bin[bin_index].count++; //-pi/2 -> pi/2
        if(bin[bin_index].empty == true)
        {
            bin[bin_index].empty = false;
        }
        //put the points into a list
        x[bin_index].push_back(lines[i][0]);
        x[bin_index].push_back(lines[i][2]);
        y[bin_index].push_back(lines[i][1]);
        y[bin_index].push_back(lines[i][3]);
    }

    for(int i=0; i<bin_size; i++)
    {
        //sort the list of points
        std::sort(x[i].begin(), x[i].end());
        std::sort(y[i].begin(), y[i].end());

        //use the endpoints for a new line
        if(bin[i].empty == false)
        {
            if(i > bin_size/2)
            {
                bin[i].lower = Point(x[i][0],y[i][0]);
                bin[i].upper = Point(x[i][x[i].size()-1],y[i][x[i].size()-1]);
            }
            else
            {
                bin[i].lower = Point(x[i][x[i].size()-1],y[i][0]);
                bin[i].upper = Point(x[i][0],y[i][x[i].size()-1]);
            }
        }
    }
}

vector<Eigen::Vector2d> get_route(int bin_size, line_bin *bin)
{

    vector<Eigen::Vector2d> route;
    int current_index;
    Eigen::Vector2d current_point(0,0);


    //find biggest y coordinate(closest to bottom)
    for(int i=0; i<bin_size; i++)
    {
        if(bin[i].upper.y > current_point[1] && bin[i].empty == false) //[1] = y coordinate
        {
            current_index = i;
            current_point[0] = bin[i].upper.x;
            current_point[1] = bin[i].upper.y;
        }
    }
    Eigen::Vector2d target_point (bin[current_index].lower.x, bin[current_index].lower.y);
    //std::cout << "cur: " << current_point << std::endl;
    //std::cout << "tar: " << target_point << std::endl;

    Eigen::Vector2d dir = target_point-current_point;
    dir = dir/dir.norm();

    bool done = false;
    //std::cout << "dir: "<< std::endl << dir << std::endl;

    //while(done == false)
    for(int i=0; i<100; i++)
    {
        route.push_back(current_point);

        Eigen::Vector2d step = dir*STEPSIZE;
        current_point += step;
        //check if any points are close
        for(int i=0; i<bin_size; i++)
        {
            if(bin[i].empty == false && i != current_index) //[1] = y coordinate
            {
                int swapdist = SWAPDIST;
                if(dir.dot(target_point-current_point) < 0)
                {
                    swapdist += OFFTRACK;
                }

                if((current_point-convert(bin[i].upper)).squaredNorm() < swapdist)
                {
                    //dir = target_point-current_point;
                    //dir = dir/dir.norm();
                    Eigen::Vector2d new_dir = convert(bin[i].lower)-current_point;
                    new_dir = new_dir/new_dir.norm();
                    if(acos((new_dir.dot(dir))/(new_dir.norm()*dir.norm())) < 2 ) //radians
                    {
                        dir = new_dir;
                        target_point = convert(bin[i].lower);
                    }

                }
                if((current_point-convert(bin[i].lower)).squaredNorm() < swapdist)
                {
                    //dir = target_point-current_point;
                    //dir = dir/dir.norm();
                    Eigen::Vector2d new_dir = convert(bin[i].upper)-current_point;
                    new_dir = new_dir/new_dir.norm();
                    if(acos((new_dir.dot(dir))/(new_dir.norm()*dir.norm())) < 2 ) //radians
                    {
                        dir = new_dir;
                        target_point = convert(bin[i].upper);
                    }
                }
            }
        }

        /*if((current_point-target_point).squaredNorm() < SWAPDIST)
        {
            done = true;
        }*/
    }
    return route;
}

Mat simple_threshold(Mat image, int thresholdGray)
{
    Mat binaryIm = Mat::zeros( image.size(), CV_8UC1 );
    //threshold(image,binaryIm, thresholdGray, 255, 0);
    inRange(image, cv::Scalar(0,0,0), cv::Scalar(thresholdGray,thresholdGray,thresholdGray), binaryIm);
    //bitwise_not ( binaryIm, binaryIm );
    return binaryIm;
}

//assumes 1280x720 picture 32x18 sections
Mat discrete_threshold(Mat image, int thresholdGray, int light)
{
    //convert to gray
    Mat gray;
    cvtColor(image, gray, CV_BGR2GRAY);

    Mat binaryIm = Mat::zeros( image.size(), CV_8UC1 ); //gray


    int splitW = IMAGE_W/SECTION;
    int splitH = IMAGE_H/SECTION;
    double min, max;
    double adjusted_threshold;

    for(int i=0; i<splitH*splitW ; i++)
    {
        Mat part (gray, Rect(SECTION*(i%splitW), SECTION*(i/splitW), SECTION, SECTION) );

        cv::minMaxLoc(part, &min, &max);
        adjusted_threshold = max*light/255 + thresholdGray;
        //threshold(part, part, adjusted_threshold, 255, 1);
        inRange(part, 0, adjusted_threshold, part);

        Rect roi(SECTION*(i%splitW), SECTION*(i/splitW), SECTION, SECTION);
        part.copyTo( binaryIm(roi) );
    }

    return binaryIm;
}
