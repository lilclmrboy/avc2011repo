#include <cv.h>
#include <highgui.h>

using namespace cv;

int main( int argc, char** argv )
{

  VideoCapture cap(0);
  if(!cap.isOpened())
	return -1;

  double fps = cap.get(CV_CAP_PROP_FPS);
  if(fps == 0)
    fps = 30;
  int wait = 1000 / fps;

  printf("Frame Rate: %f\nWait: %d", fps, wait);
  
  namedWindow( "Display Camera", CV_WINDOW_AUTOSIZE );
  for(;;) {
	Mat frame;
        cap >> frame;
 	
	imshow( "Display Image", frame );
  	if(waitKey(wait) >= 0) break;;
  }

  return 0;
}

