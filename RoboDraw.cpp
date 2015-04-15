/**
 * @file    RoboDraw.cpp
 * @author  Josh Greenberg
 * @brief   Facial detection for robotic drawing applications.
 */

#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

/** Function Headers */
void detectAndDisplay(Mat frame);

/** Global Variables */
String face_cascade_name = "lbpcascade_frontalface.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
String window_name = "Capture - Face detection";

/**
 * @function    main
 */
int main(void) {
    VideoCapture capture;
    Mat frame;
    Mat out_frame;
    Mat bgmask;
    bool pause = false;

    //-- Load the cascades
    if(!face_cascade.load(face_cascade_name)) {
        printf("--(!)Error loading face cascade\n");
        return -1;
    };
    if(!eyes_cascade.load(eyes_cascade_name)) {
        printf("--(!)Error loading eyes cascade\n");
        return -1;
    };

    //-- Run the detection and control loop
    while(!pause) {
        capture.read(frame);
        if(frame.empty()) {
            printf("--(!) No captured frame -- Break!");
            break;
        }

        //-- Apply the classifier to the frame
        detectAndDisplay(frame);

        //-- Bail out if escape was pressed
        int c = waitKey(10);
        if((char)c == 27) {
            break;
        }
        //-- Freeze frame if spacebar was pressed
        if((char)c == 32) {
            if(!frame.empty()) {
                pause = true;
                break;
            }
        }
    }

    return 0;
}

void detectAndDisplay(Mat frame) {
    std::vector<Rect> faces;
    Mat frame_gray;
    Mat frame_gray_inv;
    Mat frameBilateral;
    Mat edges;
    Mat cartoonMat;
    Mat mask;

    const int EDGES_THRESHOLD = 50;
    const int LAPLACIAN_FILTER_SIZE = 5;
    const int MEDIAN_BLUR_FILTER_SIZE = 7;
    const int FACE_BB_THICKNESS = 5;

    //-- Convert image to grayscale
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

    medianBlur(frame_gray, frame_gray, MEDIAN_BLUR_FILTER_SIZE);

    //-- Laplacian edge detection to produce hand-like sketches
    Laplacian(frame_gray, edges, CV_8U, LAPLACIAN_FILTER_SIZE);

    threshold(edges, mask, EDGES_THRESHOLD, 255, THRESH_BINARY_INV);

    //-- Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0, Size(80,80));

    for(size_t i=0; i<faces.size(); i++) {
        Mat faceROI = frame_gray(faces[i]);
        std::vector<Rect> eyes;

        //-- In each face, detect eyes
        eyes_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 |CASCADE_SCALE_IMAGE, Size(30,30));
        if(eyes.size() == 2) {
            //-- Draw the face bounding rectangle
            Point top_left(faces[i].x - faces[i].width/8, faces[i].y - faces[i].height/3);
            Point bot_right(faces[i].x + faces[i].width + faces[i].width/8, faces[i].y + faces[i].height + faces[i].height/3);
            rectangle(mask, top_left, bot_right, 0, FACE_BB_THICKNESS);

            //-- Draw the eyes bounding circles
            for(size_t j=0; j<eyes.size(); j++) {
                Point eye_center(faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2);
                int radius = cvRound((eyes[j].width + eyes[j].height)*0.25);
                circle(mask, eye_center, radius, Scalar(0,0,0), 5, 8, 0);
            }
        }
    }
    imshow(window_name, faceROI);
}
