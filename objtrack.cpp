// bob sang 2015

#include <jni.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <string.h>
#include <vector>
#include <android/log.h>

using namespace std;
using namespace cv;

extern "C" {

#define LOG_TAG "robot"

#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, fmt, ##args)


int 	first = true;
int 			depth_of_img, channels_of_img;

int 			param_display_stage = 7;
int 			param_moving_avg_wt = 2;
int 			param_detect_threshold = 20;
int 			param_min_obj_size = 2;
int 			param_dilation_amt = 30;
int 			param_erosion_amt = 10;
int 			param_brightness_factor = 50;
int 			param_contrast_factor = 0;
const char 		*param_command_on_motion;
bool 			param_is_file_mode = false;
int 			param_proc_delay = 1000;
const int 		MAX_PROC_DELAY = 1000;
const int 		MIN_PROC_DELAY = 100;
Mat img_moving_avg;

Mat prev_img;
Mat next_img;
Mat cur_img;

int detectDoor(Mat & img_rgba, int & door_x, int & door_y)
{
	Mat img_temp;
	Mat img_work;
	Mat img_gray;
	int sel_id;
	int max_area;

	Mat img_3;
    Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(2,2));

	int h;
	int s;
	int v;

	door_x = 0;
	door_y = 0;

	h = 237/2;
	s = 100*2 + 55;
	v = 25*2 + 55;

	cv::cvtColor(img_rgba, img_temp, CV_RGB2HSV);
    inRange(img_temp, Scalar(266/2-20, 0, 0), Scalar(266/2+20, 255, 255), img_work);

    cv::blur(img_work, img_gray, Size(30, 30));

//   cvtColor(img_gray, img_3, COLOR_GRAY2RGB);
//    cv::cvtColor(img_3, img_rgba , CV_RGB2RGBA);

	threshold(img_gray, img_gray, 100, 255, CV_THRESH_BINARY);
    erode(img_gray, img_gray, kernel_ero);


	Size sz;
	vector<vector<Point> > contours;
    vector<vector<Point> > contours0;
    vector<Vec4i> hierarchy;

    findContours( img_gray, contours0, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    int idx = 0;

    sel_id = 0;
    max_area = 0;
    for( idx = 0; idx < contours0.size(); idx++ )
    {
    	Rect r = boundingRect(contours0[idx]);
        Scalar color( 255, 0, 0 );
        if(hierarchy[idx][2] < 0) //Check if there is a child contour
           rectangle(img_rgba,Point(r.x-10,r.y-10), Point(r.x+r.width+10,r.y+r.height+10), Scalar(0,0,255),2,8,0); //Opened contour
         else
           rectangle(img_rgba,Point(r.x-10,r.y-10), Point(r.x+r.width+10,r.y+r.height+10), Scalar(0,255,0),2,8,0); //closed contour
//        drawContours( img_rgba, contours0, idx, color, 2, 8);

        if( max_area < r.width * r.height)
        {
        	max_area = r.width * r.height;
        	sel_id = idx;
        }
    }
    if( max_area > 400 )
    {
    	Rect r = boundingRect(contours0[sel_id]);
    	door_x = r.x + r.width/2;
    	door_y = r.y + r.height/2;
        circle(img_rgba,Point(door_x,door_y),20, Scalar(255,0,0), 3, 8, 0); //closed contour
    }
/*
    if(contours0.size() == 1)
    {
       	Rect r = boundingRect(contours0[0]);

        //LOGI("rect center: (%d,%d) w:%d h:%d\n", r.x, r.y, r.width, r.height);
       	if( r.width * r.height > 100)
       	{
       		door_x = r.x + r.width/2;
       		door_y = r.y + r.height/2;

            circle(img_rgba,Point(door_x,door_y),20, Scalar(255,0,0), 3, 8, 0); //closed contour
       	}
    }
*/
	return 0;
}


JNIEXPORT jintArray JNICALL Java_com_example_balltrace_BallTrace_DoorDetect(JNIEnv* env, jobject thiz,
		jint width, jint height, jlong yuv, jlong rgba, jlong outPtr)
{
	jint	outArray[3];
	Mat mGray = *((Mat*)yuv);
	Mat img_rgba = *((Mat*)rgba);

	Mat* mOut = (Mat*) outPtr;

	Mat img_color;
	Mat img_work;
	Mat img_smooth;
	Mat img_edge_color;
	Mat img_edge_gray;
	Mat img_diff;
	Mat img_gray;
	Mat img_contour;
	Mat img_temp;
	Mat img_2;
	Mat img_3;

	Mat img_out;
    Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(2,2));

	outArray[0] = 0;
	outArray[1] = 0;
	outArray[2] = 0;

	LOGI("aaa");
//	cv::cvtColor(img_rgba , img_color , CV_RGBA2RGB);

	int h;
	int s;
	int v;

	h = 237/2;
	s = 100*2 + 55;
	v = 25*2 + 55;

	cv::cvtColor(img_rgba, img_temp, CV_RGB2HSV);
    LOGI("img_temp: %d-%d", img_temp.depth(), img_temp.channels());
    inRange(img_temp, Scalar(h-30, s-25, v-85), Scalar(h+30, s+25, v+65), img_work);
    LOGI("inRange img_work:%d-%d", img_work.depth(), img_work.channels());

	cv::blur(img_work, img_gray, Size(30, 30));

	threshold(img_gray, img_gray, 100, 255, CV_THRESH_BINARY);
    erode(img_gray, img_gray, kernel_ero);

    cvtColor(img_gray, img_3, COLOR_GRAY2RGB);
    cv::cvtColor(img_3, img_rgba , CV_RGB2RGBA);

	Size sz;
	vector<vector<Point> > contours;
    vector<vector<Point> > contours0;
    vector<Vec4i> hierarchy;

    findContours( img_gray, contours0, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    LOGI("ffcount %d", contours0.size());

//    contours.resize(contours0.size());
//    for( size_t k = 0; k < contours0.size(); k++ )
//        approxPolyDP(Mat(contours0[k]), contours[k], 3, true);

    // iterate through all the top-level contours,
    // draw each connected component with its own random color
    int idx = 0;

    for( idx = 0; idx < contours0.size(); idx++ )
    {
    	Rect r = boundingRect(contours0[idx]);
        Scalar color( 255, 0, 0 );
        if(hierarchy[idx][2] < 0) //Check if there is a child contour
           rectangle(img_rgba,Point(r.x-10,r.y-10), Point(r.x+r.width+10,r.y+r.height+10), Scalar(0,0,255),2,8,0); //Opened contour
         else
           rectangle(img_rgba,Point(r.x-10,r.y-10), Point(r.x+r.width+10,r.y+r.height+10), Scalar(0,255,0),2,8,0); //closed contour

//        drawContours( img_rgba, contours0, idx, color, 2, 8);//, hierarchy, 0, Point() );
    }
	//cv::cvtColor(img_, img_rgba , CV_GRAY2RGBA);

//    cv::cvtColor(img_smooth , img_rgba , CV_RGB2RGBA);

//    cv::cvtColor(img_edge_color, img_rgba, CV_RGB2RGBA);


    jintArray retArray = env->NewIntArray(3);
    env->SetIntArrayRegion(retArray, 0 , 3, outArray);

    return retArray;
}

//Check if there is motion in the result matrix
// count the number of changes and return.
inline int detectMotion(const Mat & motion, Mat & result, Mat & result_cropped,
                 int x_start, int x_stop, int y_start, int y_stop,
                 int max_deviation,
                 Scalar & color)
{
    int noc = 0;
    int c;
	double sd;
	int isd;

    // calculate the standard deviation
    Scalar mean, stddev;
    meanStdDev(motion, mean, stddev);
	sd = stddev.val[0];
	isd = int(sd);
//	sd = static_cast<double>(stddev.at<uchar>(0,0));
    // if not to much changes then the motion is real (neglect agressive snow, temporary sunlight)
//    if(stddev[0] < max_deviation)
    c = 0;
    {
        int min_x = motion.cols, max_x = 0;
        int min_y = motion.rows, max_y = 0;
        // loop over image and detect changes
        for(int j = y_start; j < y_stop; j+=2)
        { // height
            for(int i = x_start; i < x_stop; i+=2)
            { // width
                // check if at pixel (j,i) intensity is equal to 255
                // this means that the pixel is different in the sequence
                // of images (prev_frame, current_frame, next_frame)
            	c++;
                if(static_cast<int>(motion.at<uchar>(j,i)) == 255)
                {
                    noc++;
                    if(min_x>i) min_x = i;
                    if(max_x<i) max_x = i;
                    if(min_y>j) min_y = j;
                    if(max_y<j) max_y = j;
                }
            }
        }
#if 0		
        if(number_of_changes){
            //check if not out of bounds
            if(min_x-10 > 0) min_x -= 10;
            if(min_y-10 > 0) min_y -= 10;
            if(max_x+10 < result.cols-1) max_x += 10;
            if(max_y+10 < result.rows-1) max_y += 10;
            // draw rectangle round the changed pixel
            Point x(min_x,min_y);
            Point y(max_x,max_y);
            Rect rect(x,y);
            Mat cropped = result(rect);
            cropped.copyTo(result_cropped);
            rectangle(result,rect,color,1);
        }
#endif
		LOGI("x:%d-%d y:%d-%d std:0 noc:%d c:%d sd:%d", x_start, x_stop, y_start, y_stop, noc, c, isd);
        return noc;
    }
    return 0;
}


JNIEXPORT jintArray JNICALL Java_com_example_balltrace_BallTrace_MotionDetect(JNIEnv* env, jobject thiz,
		jint width, jint height, jlong yuv, jlong rgba, jlong outPtr)
{
	jint	outArray[3];
	Mat mGray = *((Mat*)yuv);
	Mat img_rgba = *((Mat*)rgba);

	Mat* mOut = (Mat*) outPtr;

	Mat img_color;
	Mat img_work;
	Mat img_gray;
	
	Mat d1, d2, motion;
	Mat result, result_cropped;

    int number_of_changes, number_of_sequence = 0;
    Scalar mean_, color(0,255,255); // yellow
    
    // Detect motion in window
    int x_start = 10, x_stop = img_rgba.cols-11;
    int y_start = 350, y_stop = 530;
	int max_deviation = 20;
	//y_start = 10;
	//y_stop = 600;//img_rgba.rows - 11;

	//LOGI("00000000000000000000");
    // Erode kernel

    Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(2,2));
	
   
	outArray[0] = 0;
	outArray[1] = 0;
	outArray[2] = 0;

	//img_rgba.convertTo(img_color, 3);
	//LOGI("aaa");
	cv::cvtColor(img_rgba , img_color , CV_RGBA2RGB);	
	cv::cvtColor(img_color, img_gray, CV_RGB2GRAY);
	
	if( first )
	{
		img_gray.copyTo(prev_img);
		img_gray.copyTo(next_img);
		img_gray.copyTo(cur_img);
		first = 0;
	}
	else
	{
		prev_img = cur_img;
		cur_img = next_img;
		next_img = img_gray;
	}
	
	absdiff(prev_img, next_img, d1);
	absdiff(next_img, cur_img, d2);

    bitwise_and(d1, d2, motion);
//    LOGI("DDD");
    threshold(motion, motion, 35, 255, CV_THRESH_BINARY);
//    LOGI("EEE");
    erode(motion, motion, kernel_ero);
//    LOGI("FFFF");
//	cv::blur(motion, motion, Size(5, 30));

	//cv::dilate(motion, motion, Mat(), Point(-1, -1), param_dilation_amt+2);
	//LOGI("E2");

	//cvErode(img_gray, img_gray, 0, param_erosion_amt);			// erode again to get some of the original proportion back
	//cv::erode(motion, motion, Mat(), Point(-1, -1), param_erosion_amt, 1, 1);

	vector<vector<Point> > contours;
    vector<vector<Point> > contours0;
    vector<Vec4i> hierarchy;

//    findContours( motion, contours0, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//     LOGI("FFFF");
//    LOGI("ffcount %d", contours0.size());

    cvtColor(motion, img_rgba, CV_GRAY2RGBA);

    number_of_changes = detectMotion(motion, result, result_cropped,  x_start, x_stop, y_start, y_stop, max_deviation, color);
	//LOGI("num of changes :%d", number_of_changes);
		
    jintArray retArray = env->NewIntArray(3);
    env->SetIntArrayRegion(retArray, 0 , 3, outArray);

    return retArray;

}


JNIEXPORT jintArray JNICALL Java_com_example_balltrace_BallTrace_MotionDetect2(JNIEnv* env, jobject thiz,
		jint width, jint height, jlong yuv, jlong rgba, jlong outPtr)
{
	jint	outArray[3];
	Mat mGray = *((Mat*)yuv);
	Mat img_rgba = *((Mat*)rgba);

	Mat* mOut = (Mat*) outPtr;

	Mat img_color;
	Mat img_work;
	Mat img_smooth;
	Mat img_edge_color;
	//(Size(width, height), CV_8UC4);
	Mat img_edge_gray;
	Mat img_diff;
	Mat img_gray;
	Mat img_contour;
	Mat img_temp;

	Mat img_out;

	outArray[0] = 0;
	outArray[1] = 0;
	outArray[2] = 0;

	//img_rgba.convertTo(img_color, 3);
	LOGI("aaa");
	cv::cvtColor(img_rgba , img_color , CV_RGBA2RGB);

//	cvtColor(img_color, img_gray, CV_RGB2GRAY);

	cv::resize(img_color, img_work, Size(800,600));
//	img_work = img_color;

	// smoothen the image
	// cvSmooth(img_work, img_smooth, CV_GAUSSIAN, 7);
	//cvSmooth(img_work, img_smooth, CV_BILATERAL, 5, 5, 30, 30);
	cv::blur(img_work, img_smooth, Size(5, 30));

	// increase contrast and adjust brightness
	cv::addWeighted(img_smooth, 1, img_smooth, 1, param_brightness_factor-50, img_color);
	// increase contrast further if specified
	for(int contrast_idx = 0; contrast_idx < param_contrast_factor; contrast_idx++) {
		cv::addWeighted(img_color, 1, img_color, 1, 0, img_color);
	}
	
	cv::Laplacian(img_color, img_edge_color, CV_8U, 3, 1, 0);
	
   	cv::cvtColor(img_edge_color , img_rgba , CV_RGB2RGBA);

	cvtColor(img_color, img_edge_gray, CV_RGB2GRAY);
	//cv::Laplacian(img_edge_gray, img_edge_gray, CV_8U, 3, 1, 0);

	threshold(img_edge_gray, img_edge_gray, 25+param_detect_threshold, 255, CV_THRESH_BINARY);
	Size sz;

	cvtColor(img_edge_gray, img_edge_color, COLOR_GRAY2RGB);
	sz = img_edge_color.size();

	//LOGI("img_edge_color.depth=%d channels:%d size(%d,%d)\n", img_edge_color.depth(), img_edge_color.channels(), sz.width, sz.height);

	//LOGI("DDDD");
	if( (img_edge_color.size() == img_work.size()) && (img_edge_color.channels() == img_work.channels()) )
	{
		cv::add(img_edge_color, img_color, img_color);
	}


	if (first) {
		img_moving_avg = Mat::zeros(img_color.size(), CV_32FC3);
		img_color.convertTo(img_moving_avg, CV_32FC3);
		first = false;
		LOGI("First");
	}
	else {
		cv::accumulateWeighted(img_color, img_moving_avg, 0.05);
	}

	//LOGI("DDDD");
//	sz = img_moving_avg.size();
//	LOGI("avg: depth=%d channels:%d size:(%d,%d) depth=%d %d",img_moving_avg.depth(),img_moving_avg.channels(),	sz.width, sz.height,img_color.depth(),img_color.channels());

// convert the moving avg to a format usable for diff
	img_moving_avg.convertTo(img_temp, CV_8UC3);


	cv::absdiff(img_color, img_temp, img_diff);
	// subtract current from moving average.

	cvtColor(img_diff, img_gray, CV_RGB2GRAY);
	// convert image to gray


	threshold(img_gray, img_gray, 25+param_detect_threshold, 255, CV_THRESH_BINARY);	// convert image to black and white
	//cvThreshold(greyImage, greyImage, 70, 255, CV_THRESH_BINARY);	//Convert the image to black and white.
	//cvAdaptiveThreshold(img_gray, img_gray, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 0);

	LOGI("EEEE");
 	// dilate and erode to reduce noise and join irregular blobs
//	cvErode(img_gray, img_gray, 0, 2); 							// erode to remove noise
	cv::erode(img_gray, img_gray, Mat(), Point(-1, -1), 2, 1, 1);

	//LOGI("E1");

	//cvDilate(img_gray, img_gray, 0, param_dilation_amt+2);		// dilate to join and fill blobs
	cv::dilate(img_gray, img_gray, Mat(), Point(-1, -1), param_dilation_amt+2);
	//LOGI("E2");

	//cvErode(img_gray, img_gray, 0, param_erosion_amt);			// erode again to get some of the original proportion back
	cv::erode(img_gray, img_gray, Mat(), Point(-1, -1), param_erosion_amt, 1, 1);
	LOGI("E3");

//	cvConvertScale(img_gray, img_contour, 1.0, 0.0);			// copy image to the contour image for contour detection
	img_contour = img_gray;

	// find the contours of the moving images in the frame.
//	cvClearMemStorage(mem_store);
//	CvSeq* contour = 0;
//	cvFindContours(img_contour, mem_store, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	//LOGI("E4");
	vector<vector<Point> > contours;
    vector<vector<Point> > contours0;
    vector<Vec4i> hierarchy;

    findContours( img_contour, contours0, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//     LOGI("FFFF");
    LOGI("ffcount %d", contours0.size());

	bool motion_detected = false;
//    contours.resize(contours0.size());
//    for( size_t k = 0; k < contours0.size(); k++ )
//        approxPolyDP(Mat(contours0[k]), contours[k], 3, true);

    // iterate through all the top-level contours,
    // draw each connected component with its own random color
    int idx = 0;
//    for( ; idx >= 0; idx = hierarchy[idx][0] )
    for( idx = 0; idx < contours0.size(); idx++ )
    {
        Scalar color( 0, 0, 255 );
        drawContours( img_color, contours0, idx, color, 2, 8);//, hierarchy, 0, Point() );
        //LOGI("Counter %d ccccccccc", idx);

    }
    circle( img_color, Point(200,400), 100, Scalar(255,0,255), 3, 8, 0 );
//	cv::cvtColor(img_gray, img_rgba , CV_GRAY2RGBA);

//    cv::cvtColor(img_smooth , img_rgba , CV_RGB2RGBA);

//    cv::cvtColor(img_edge_color, img_rgba, CV_RGB2RGBA);
//    cvtColor(img_gray, img_rgba, CV_GRAY2RGBA);
/*
	// process each moving contour in the current frame...
	for (; contour != 0; contour = contour->h_next) {
		CvRect bnd_rect = cvBoundingRect(contour, 0);	// get a bounding rect around the moving object.

		// discard objects smaller than our expected object size
		int obj_size_pct = bnd_rect.width * bnd_rect.height * 100 / (sz_of_img.height * sz_of_img.width);
		if (obj_size_pct < param_min_obj_size) continue;

		// either draw the contours or motion detection marker
		if(6 == param_display_stage) {
			cvDrawContours(img_work, contour, CV_RGB(0,255,0), CV_RGB(0,255,0), 3, CV_FILLED);
		}
		else if(7 == param_display_stage) {
			CvPoint center;
			center.x = bnd_rect.x + bnd_rect.width/2;
			center.y = bnd_rect.y + bnd_rect.height/2;
			int rad = (bnd_rect.width < bnd_rect.height ? bnd_rect.width : bnd_rect.height)/2;

			while (rad > 0) {
				cvCircle(img_work, center, rad, CV_RGB(153,204,50), 1, CV_AA);
				rad -= 8;
			}
		}
		motion_detected = true;
	}

	if(is_action_mode) {
		if(motion_detected) {
			cvReleaseCapture(&inp_device);
			system(param_command_on_motion);
			inp_device = capture_input(argc, argv);
		}
	}
	else {
		char c = display_frame(motion_detected);
		if(27 == c) break;
		else if('s' == c) save_settings();
		else if('r' == c) load_settings();
	}




*/
    //*mOut = img_color.clone();

    jintArray retArray = env->NewIntArray(3);
    env->SetIntArrayRegion(retArray, 0 , 3, outArray);

    return retArray;

}

JNIEXPORT jintArray JNICALL Java_com_example_balltrace_BallTrace_BallDetect(JNIEnv* env, jobject thiz,
		jint width, jint height, jlong yuv, jlong rgba, jlong outPtr)
{
	jint	outArray[5];
	Mat mGray = *((Mat*)yuv);
	Mat mRgba = *((Mat*)rgba);

	int door_x = 0;
	int door_y = 0;

	Mat hsv_frame;

	outArray[0] = 0;
	outArray[1] = 0;
	outArray[2] = 0;
	outArray[3] = -1;	// door rect center X
	outArray[4] = -1;	// door rect center y

	detectDoor(mRgba, door_x, door_y);

	outArray[3] = door_x;
	outArray[4] = door_y;

	//Mat thresholded;
	cv::Mat* thresholded = new cv::Mat(cv::Size(width, height), CV_8UC1);
	cv::Rect roi( cv::Point( 0, 0 ), Size(width, height) );
    CvSize size = cvSize(width, height);
    cvtColor(mRgba, hsv_frame, CV_RGB2HSV);

    inRange(hsv_frame, Scalar(1, 80, 80), Scalar(7, 250, 250), *thresholded);

    GaussianBlur(*thresholded, *thresholded, Size(9,9), 0, 0);
    vector<Vec3f> circles;
    HoughCircles(*thresholded, circles, CV_HOUGH_GRADIENT,1.5, height/4, 100, 40, 15, 80 );

    for( size_t i = 0; i < circles.size(); i++ )
    {
    	Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        if( radius < 5 )
        	continue;
        // draw the circle center
        circle( mRgba, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // draw the circle outline
        circle( mRgba, center, radius, Scalar(0,0,255), 3, 8, 0 );
        outArray[0] = cvRound(circles[i][0]);
        outArray[1] = cvRound(circles[i][1]);
        outArray[2] = radius;
        break;
    }

    delete thresholded;

    jintArray retArray = env->NewIntArray(5);
    env->SetIntArrayRegion(retArray, 0 , 5, outArray);

    return retArray;
}

}// extern C
