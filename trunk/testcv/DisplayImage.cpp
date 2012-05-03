#include <cv.h>
#include <highgui.h>
#include <math.h>

using namespace cv;

int main( int argc, char** argv )
{
  VideoCapture cap;

  if(argc == 1)
	cap.open(0);
  else
	cap.open(argv[1]);
  
  if(!cap.isOpened())
	return -1;

	CvSize frame_size;
	frame_size.height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	frame_size.width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	
	
	
  double fps = cap.get(CV_CAP_PROP_FPS);
  if(fps == 0)
    fps = 30;
  int wait = 1000 / fps;

  printf("Frame Rate: %f\nWait: %d\n", fps, wait);
	
	 namedWindow( "converted", CV_WINDOW_AUTOSIZE );
	
	vector<Point2f> frame1_features;
	int number_of_feature_rows = 20;
	int number_of_feature_cols = 20;
	int number_of_features = number_of_feature_rows * number_of_feature_cols;
	int height_inc = frame_size.height / number_of_feature_rows;
	int width_inc = frame_size.width / number_of_feature_cols;
	// Fill the initial feature vector.
	for (int i = 1; i < number_of_feature_rows; i++) {
		for (int j = 1; j < number_of_feature_cols; j++) {
			frame1_features.push_back(Point2f(width_inc * j, height_inc * i));
		}
	}
	
	
	for(;;) {
		Mat tmpM;
		
		Mat frame, frame1, frame1_1c, frame2_1c, pyramid1, pyramid2;
        
		
		
		vector<Point2f> frame2_features;
		vector<float> err;
		vector<uchar> status;
		
		cap >> frame;
		
		cvtColor(frame, frame1_1c, CV_BGR2GRAY);
 	
		frame1 = frame.clone();
		
		cap >> frame;
		
		cvtColor(frame, frame2_1c, CV_BGR2GRAY);
		
		calcOpticalFlowPyrLK(frame1_1c, frame2_1c, 
												 frame1_features, frame2_features, status, err);
		
		for (int i = 0; i < number_of_features; ++i) {
			if (status[i] == 0) continue;
			
			int line_thickness = 1;
			Scalar line_color = CV_RGB(255, 0, 0);
			
			Point p,q;
			
			p.x = (int) frame1_features[i].x;
			p.y = (int) frame1_features[i].y;
			q.x = (int) frame2_features[i].x;
			q.y = (int) frame2_features[i].y;
			
			double angle = atan2((double) p.y - q.y, (double) p.x - q.x);
			double hypotenuse = sqrt( (p.y - q.y)*(p.y - q.y) + (p.x - q.x) * (p.x - q.x) );
			q.x = (int) p.x - 3 * hypotenuse * cos(angle);
			q.y = (int) p.y - 3 * hypotenuse * sin(angle);
			
			line( frame1, p, q, line_color, line_thickness, CV_AA, 0);
			
			p.x = (int) (q.x + 9 * cos(angle + 3.1415 / 4));
			p.y = (int) (q.y + 9 * sin(angle + 3.1415 / 4));
			line( frame1, p, q, line_color, line_thickness, CV_AA, 0);
			p.x = (int) (q.x + 9 * cos(angle - 3.1415 / 4));
			p.y = (int) (q.y + 9 * sin(angle - 3.1415 / 4));
			line( frame1, p, q, line_color, line_thickness, CV_AA, 0);
			
		}
		
		imshow( "converted", frame1);
  	if(waitKey(wait) >= 0) break;
  }

  return 0;
}

