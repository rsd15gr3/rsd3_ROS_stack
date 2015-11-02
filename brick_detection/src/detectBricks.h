struct brick {
	Point center;
	Point orientation;
}red, yellow, blue;


//Counts how many pixels in image img that has value val, within the rectangle defined by points pt1 and pt2
int countpixelVal(Mat img, int val, Point pt1, Point pt2){
    int pixels=0;
    for(int x=pt1.x;x< pt2.x;x++){
        for(int y=pt1.y; y<pt2.y;y++){
            if((int)img.at<uchar>( y,x)==val){
                pixels++;
            }
        }
    }
    return pixels;
}

//Calculates center of mass for all pixelvalues val in image src
Point centerOfMass(Mat src, int val){
    Point p;
    p.x=0;
    p.y=0;
    int numPixels = countpixelVal(src, val,Point(0,0),Point(src.cols-1,src.rows-1));
    int sumX=0, sumY=0;
    if(numPixels>0){
        for(int x=0;x< src.cols;x++){
            for(int y=0; y<src.rows;y++){
                if((int)src.at<uchar>(y, x)==val){
                    sumX=sumX+x;
                    sumY=sumY+y;
                }
            }
        }
        //std::cout<<sumX<<", "<<numPixels<<std::endl;
        p.x=sumX/numPixels;
        p.y=sumY/numPixels;
    }
    return p;
}

//Detects bricks
void detectBrick(Mat src){
	//namedWindow( "Color", CV_WINDOW_NORMAL );
	//imshow("Color",src);

	Mat dst=src;
	//Closing of the image and medianblur helps smooth away light reflections on bricks
	int i=10;
	Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*i + 1, 2*i + 1 ), Point( i, i ));
	morphologyEx( src, dst, MORPH_CLOSE, element);
	medianBlur( src, dst, 5*i+1);
	//namedWindow( "Blur", CV_WINDOW_NORMAL );
	//imshow("Blur",dst);
	//waitKey();
	//destroyAllWindows();
	vector<Mat> channels(3);
	split(dst,channels);
	Mat r,g,b;
	b=channels[0];
	g=channels[1];
	r=channels[2];

	//Tries to get rid of light reflections on the conveyor belt to avoid confusing it for bricks
	for(int x=0;x< dst.cols;x++){
		for(int y=0; y<dst.rows;y++){
			if((int)r.at<uchar>(y,x)>159&&(int)b.at<uchar>(y,x)>159&&(int)g.at<uchar>(y,x)>159){
				r.at<uchar>(y,x)=0;
				b.at<uchar>(y,x)=0;
				g.at<uchar>(y,x)=0;
			}
		}
	}

	//Improves red, green and blue channels for finding bricks by looking at the other channels
	for(int x=0;x< dst.cols;x++){
		for(int y=0; y<dst.rows;y++){
			if((int)b.at<uchar>(y,x)>191||(int)g.at<uchar>(y,x)>191){
				r.at<uchar>(y,x)=0;
			}
			if((int)b.at<uchar>(y,x)>191||(int)r.at<uchar>(y,x)>191){
				g.at<uchar>(y,x)=0;
			}
			if((int)r.at<uchar>(y,x)>191||(int)g.at<uchar>(y,x)>191){
				b.at<uchar>(y,x)=0;
			}
		}
	}
	//Converts each color channel to binary image
	threshold(r, r, 191, 255, THRESH_BINARY);
	threshold(g, g, 191, 255, THRESH_BINARY);
	threshold(b, b, 191, 255, THRESH_BINARY);

	//Remove bricks that extend out the region of interest in order to avoid miscalculations
	for(int y=0;y< dst.rows;y++){
		if((int)r.at<uchar>(y,0)>0){
            floodFill(r, Point(0,y), 0);
		}
		if((int)r.at<uchar>(y,r.cols-1)>0){
		    floodFill(r, Point(r.cols-1,y), 0);
		}
		if((int)g.at<uchar>(y,0)>0){
            floodFill(g, Point(0,y), 0);
		}
		if((int)g.at<uchar>(y,g.cols-1)>0){
		    floodFill(g, Point(g.cols-1,y), 0);
		}
		if((int)b.at<uchar>(y,0)>0){
            floodFill(b, Point(0,y), 0);
		}
		if((int)b.at<uchar>(y,b.cols-1)>0){
		    floodFill(b, Point(b.cols-1,y), 0);
		}
	}

	//Fills the legobricks with individual color to seperate the and count how many detected bricks
	int red_replacementcolor=1;
	int green_replacementcolor=1;
	int blue_replacementcolor=1;
	for(int x=0;x< r.cols;x++){
        for(int y=0; y<r.rows;y++){
            if((int)r.at<uchar>(y, x)==255){
            	//fills replacement color
                floodFill(r, Point(x,y), red_replacementcolor);
                //if theres less pixels of this replacement color than 2500 then it is too small to be a brick
                if(countpixelVal(r,red_replacementcolor,Point(0,0), Point(r.cols,r.rows))<2500){
                	//therefore blacks it to erase
                	floodFill(r, Point(x,y), 0);
                }
                //else if there's more pixels than 2500 then it's a brick and replacementcolor is incremented
                else
                {
                	red_replacementcolor++;
                }
            }
            //same for green which will correspond to yellow brick
            if((int)g.at<uchar>(y, x)==255){
            	floodFill(g, Point(x,y), green_replacementcolor);
                if(countpixelVal(g,green_replacementcolor,Point(0,0), Point(g.cols,g.rows))<2500){
                	floodFill(g, Point(x,y), 0);
                }
                else
                {
                	green_replacementcolor++;
                }
           	}
            //and blue
            if((int)b.at<uchar>(y, x)==255){
            	floodFill(b, Point(x,y), blue_replacementcolor);
                if(countpixelVal(b,blue_replacementcolor,Point(0,0), Point(b.cols,b.rows))<2500){
                	floodFill(b, Point(x,y), 0);
                }
                else
                {
                	blue_replacementcolor++;
                }
           	}
        }
	}

	//Declarations of Vecf4 arrays for storing orientations of each brick
	std::vector<Vec4f> red_lines(red_replacementcolor-1);
	std::vector<Vec4f> green_lines(green_replacementcolor-1);
	std::vector<Vec4f> blue_lines(blue_replacementcolor-1);

	//Declarations of Point arrays for storing location of each brick
	std::vector<Point> red_points(red_replacementcolor-1);
	std::vector<Point> green_points(green_replacementcolor-1);
	std::vector<Point> blue_points(blue_replacementcolor-1);

	//Prints how many bricks were found of each color
	std::cout<<"Found "<<red_lines.size()<<" red lego bricks in image"<<std::endl;
	std::cout<<"Found "<<green_lines.size()<<" yellow lego bricks in image"<<std::endl;
	std::cout<<"Found "<<blue_lines.size()<<" blue lego bricks in image"<<std::endl;

	//Finds location and orientation of each brick
	if(red_replacementcolor>1){
		//The current value of replacementcolor-1 is equal to amount of bricks found
		//n increments through all detected red bricks, first brick will have pixelvalue 1, second 2, etc...
		for(int n=1;n<red_replacementcolor;n++){

			//Finds center of mass for all pixels of value n, all those pixels will belong to the same brick
			//The center of mass of pixel n is thus the location of the n'th red brick in the image
			red_points[n-1]=centerOfMass(r,n);

			//All the pixels of a particular red brick is stored as points in a point array
			int nn=0;
			int numPix = countpixelVal(r, n, Point(0,0), Point(r.cols,r.rows));
			vector<Point> m(numPix);
			for(int x=0;x< r.cols;x++){
		        for(int y=0; y<r.rows;y++){
		            if((int)r.at<uchar>(y,x)==n){
		            	m[nn]=Point(x,y);
		            	nn++;
		            }
		        }
			}
			//fitLine does linear regression on all points in the point array to find the best line through those points
			//The line will have the same orientation as the brick
			Vec4f line;
			fitLine(m,line,CV_DIST_L2,0,0.1,0.1);
			std::cout<<"Normal vector for red brick number "<<n<<" is ("<<line[0]<<", "<<line[1]<<") and the location is "<<red_points[n-1]<<std::endl;
			//puts resulting line in array
			red_lines[n-1]=line;
		}
	}
	//Same for yellow brick
	if(green_replacementcolor>1){
		for(int n=1;n<green_replacementcolor;n++){
			green_points[n-1]=centerOfMass(g,n);
			int nn=0;
			int numPix = countpixelVal(g, n, Point(0,0), Point(g.cols,g.rows));
			vector<Point> m(numPix);
			for(int x=0;x< g.cols;x++){
		        for(int y=0; y<g.rows;y++){
		            if((int)g.at<uchar>(y,x)==n){
		            	m[nn]=Point(x,y);
		            	nn++;
		            }
		        }
			}
			Vec4f line;
			fitLine(m,line,CV_DIST_L2,0,0.1,0.1);
			std::cout<<"Normal vector for yellow brick number "<<n<<" is ("<<line[0]<<", "<<line[1]<<") and the location is "<<green_points[n-1]<<std::endl;
			green_lines[n-1]=line;
		}
	}
	//...and blue
	if(blue_replacementcolor>1){
		for(int n=1;n<blue_replacementcolor;n++){
			blue_points[n-1]=centerOfMass(b,n);
			int nn=0;
			int numPix = countpixelVal(b, n, Point(0,0), Point(b.cols,b.rows));
			vector<Point> m(numPix);
			for(int x=0;x< b.cols;x++){
		        for(int y=0; y<b.rows;y++){
		            if((int)b.at<uchar>(y,x)==n){
		            	m[nn]=Point(x,y);
		            	nn++;
		            }
		        }
			}
			Vec4f line;
			fitLine(m,line,CV_DIST_L2,0,0.1,0.1);
			std::cout<<"Normal vector for blue brick number "<<n<<" is ("<<line[0]<<", "<<line[1]<<") and the location is "<<blue_points[n-1]<<std::endl;
			blue_lines[n-1]=line;
		}
	}

	//Draws each line and marks location of each brick so we can see that the code works
	double red_slope;
	double red_b;
	double green_slope;
	double green_b;
	double blue_slope;
	double blue_b;
	threshold(r, r, 0, 255, THRESH_BINARY);
	threshold(g, g, 0, 255, THRESH_BINARY);
	threshold(b, b, 0, 255, THRESH_BINARY);
	for(int n=0;n<red_lines.size();n++){
		red_slope=red_lines[n][1]/red_lines[n][0];
		red_b=red_lines[n][3]-red_slope*red_lines[n][2];
		circle(r,red_points[n],10,0,-1);
		line(r,Point(0,(int)red_b),Point(r.cols-1,(int)(red_b+red_slope*(r.cols-1))),191);
	}
	for(int n=0;n<green_lines.size();n++){
		green_slope=green_lines[n][1]/green_lines[n][0];
		green_b=green_lines[n][3]-green_slope*green_lines[n][2];
		circle(g,green_points[n],10,0,-1);
		line(g,Point(0,(int)green_b),Point(g.cols-1,(int)(green_b+green_slope*(g.cols-1))),191);
	}
	for(int n=0;n<blue_lines.size();n++){
		blue_slope=blue_lines[n][1]/blue_lines[n][0];
		blue_b=blue_lines[n][3]-blue_slope*blue_lines[n][2];
		circle(b,blue_points[n],10,0,-1);
		line(b,Point(0,(int)blue_b),Point(b.cols-1,(int)(blue_b+blue_slope*(b.cols-1))),191);
	}

	//Display results
	//namedWindow( "Blue", CV_WINDOW_NORMAL );
	//namedWindow( "Yellow", CV_WINDOW_NORMAL );
	//namedWindow( "Red", CV_WINDOW_NORMAL );
	//imshow("Blue",b);
	//imshow("Yellow",g);
	//imshow("Red",r);
	//waitKey();
	//destroyAllWindows();
}

