 #include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <zbar.h>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace zbar;

int main(int argc, char **argv) 
{

//Define camera matrix according to webcam calibration
cv::Mat_<double> cameraMatrix(3, 3);
cameraMatrix.at<double>(0,0) =  1.3442848643472917e+03;
cameraMatrix.at<double>(0,1) = 0.0;
cameraMatrix.at<double>(0,2) = 6.3950000000000000e+02;
cameraMatrix.at<double>(1,0) =  0.0;
cameraMatrix.at<double>(1,1) = 1.3442848643472917e+03;
cameraMatrix.at<double>(1,2) = 3.595e+02;
cameraMatrix.at<double>(2,0) = 0.0;
cameraMatrix.at<double>(2,1) = 0.0; 
cameraMatrix.at<double>(2,2) = 1.0;

//Define distortion parameters according to webcam calibration
cv::Mat_<double> distortionParameters(1, 5); //k1, k2, p1, p2, k3
distortionParameters.at<double>(0, 0) = 7.9440223269640672e-03;
distortionParameters.at<double>(0, 1) = -5.6562236732221527e-01;
distortionParameters.at<double>(0, 2) =  0.0;
distortionParameters.at<double>(0, 3) =  0.0;
distortionParameters.at<double>(0, 4) = 1.6991852512288661e+00;

//Taken together, the returned rotation and translation vector give the homogenious transform for camera -> object

//Taking the inverse gives the camera location relative to the object

int cam_idx = 0;

//Set camera ID from argument if there is one
if (argc == 2) 
{
cam_idx = atoi(argv[1]);
}

//open opencv camera video source
cv::VideoCapture cap(cam_idx);
if (!cap.isOpened()) 
{
cerr << "Could not open camera." << endl;
exit(EXIT_FAILURE);
}

//Make size same as calibration
cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

//Create a window to show the video and any QR codes that have been identified
cv::namedWindow("captured", CV_WINDOW_AUTOSIZE);
    
// Create a zbar reader to check for barcodes
ImageScanner scanner;
    
// Configure the reader
scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

for (;;) 
{

//Use ZBar to get QR code vertices and text.  

// Capture an OpenCV frame
cv::Mat frame, frame_grayscale;
cap >> frame;

// Convert to grayscale
cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

// Obtain image data
int width = frame_grayscale.cols;
int height = frame_grayscale.rows;
uchar *raw = (uchar *)(frame_grayscale.data);

// Wrap image data
Image image(width, height, "Y800", raw, width * height);

// Scan the image for barcodes
if(scanner.scan(image) == -1)
{
return 1; //Error occured
}

// Extract results
int counter = 0;
for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) 
{
time_t now;
tm *current;
now = time(0);
current = localtime(&now);

// do something useful with results
cout    << "[" << current->tm_hour << ":" << current->tm_min << ":" << setw(2) << setfill('0') << current->tm_sec << "] " << counter << " "
<< "decoded " << symbol->get_type_name()
<< " symbol \"" << symbol->get_data() << '"' << endl;

//Skip invalid QR codes
if (symbol->get_location_size() != 4) 
{
fprintf(stderr, "Got a QR code with more than 4 points in the polygon\n");
continue;
}
            
// Draw location of the symbols found
line(frame, cv::Point(symbol->get_location_x(0), symbol->get_location_y(0)), cv::Point(symbol->get_location_x(1), symbol->get_location_y(1)), cv::Scalar(255, 0, 0), 2, 8, 0); //Red 0->1
line(frame, cv::Point(symbol->get_location_x(1), symbol->get_location_y(1)), cv::Point(symbol->get_location_x(2), symbol->get_location_y(2)), cv::Scalar(0, 255, 0), 2, 8, 0); //Green 1 -> 2
line(frame, cv::Point(symbol->get_location_x(2), symbol->get_location_y(2)), cv::Point(symbol->get_location_x(3), symbol->get_location_y(3)), cv::Scalar(0, 0, 255), 2, 8, 0); //Blue 2 -> 3
line(frame, cv::Point(symbol->get_location_x(3), symbol->get_location_y(3)), cv::Point(symbol->get_location_x(0), symbol->get_location_y(0)), cv::Scalar(0, 255, 255), 2, 8, 0); //Yellow  3 -> 0
            
//Optionally extract QR code dimensions from embedded text   

//Convert scanner points to opencv points
std::vector<cv::Point2d> openCVPoints;
for(int i=0; i<symbol->get_location_size(); i++)
{
openCVPoints.push_back(cv::Point2d(symbol->get_location_x(i), symbol->get_location_y(i)));
}


//The center of the coordinate system associated with the QR code is in the center of the rectangle (for now assuming qr code is .2 meter x .2 meter)
std::vector<cv::Point3d> objectVerticesInObjectCoordinates;
objectVerticesInObjectCoordinates.push_back(cv::Point3d(-.06,-.06,0));
objectVerticesInObjectCoordinates.push_back(cv::Point3d(.06,-.06,0));
objectVerticesInObjectCoordinates.push_back(cv::Point3d(.06,.06,0));
objectVerticesInObjectCoordinates.push_back(cv::Point3d(-.06,.06,0));

//Make buffers to get 3x1 rotation vector and 3x1 translation vector
cv::Mat_<double> rotationVector(3,1);
cv::Mat_<double> translationVector(3,1);


//Use solvePnP to get the rotation and translation vector of the object assuming the following:
cv::solvePnP(objectVerticesInObjectCoordinates, openCVPoints, cameraMatrix, distortionParameters, rotationVector, translationVector);

printf("Translation vector: %lf %lf %lf\n", translationVector.at<double>(0, 0), translationVector.at<double>(0, 1), translationVector.at<double>(0, 2));

printf("Rotation vector: %lf %lf %lf\n", rotationVector.at<double>(0, 0), rotationVector.at<double>(0, 1), rotationVector.at<double>(0, 2));

cv::Mat_<double> rotationMatrix, viewMatrix(4, 4);

//Get 3x3 rotation matrix
cv::Rodrigues(rotationVector, rotationMatrix);

//Zero out view matrix
viewMatrix = cv::Mat::zeros(4, 4, CV_64F); 

//Get opencv transfer matrix (camera -> object)
for(int row = 0; row < 3; row++)
{

for(int col = 0; col < 3; col ++)
{
viewMatrix.at<double>(row, col) = rotationMatrix.at<double>(row, col);
}

viewMatrix.at<double>(row, 3) = translationVector.at<double>(row, 0);
}
viewMatrix.at<double>(3,3) = 1.0;


//Convert to a standard opengl tranfer matrix (http://answers.opencv.org/question/23089/opencv-opengl-proper-camera-pose-using-solvepnp/)
cv::Mat_<double> cvToGl = cv::Mat::zeros(4, 4, CV_64F); 
cvToGl.at<double>(0, 0) = 1.0f; 
cvToGl.at<double>(1, 1) = -1.0f; // Invert the y axis 
cvToGl.at<double>(2, 2) = -1.0f; // invert the z axis 
cvToGl.at<double>(3, 3) = 1.0f; 
viewMatrix = cvToGl * viewMatrix;

cv::Mat_<double> glViewMatrix = cv::Mat::zeros(4, 4, CV_64F);
cv::transpose(viewMatrix , glViewMatrix);


//Invert matrix to get position and orientation of camera relative to tag
cv::Mat_<double> positionAndOrientationOfCamera;
invert(viewMatrix, positionAndOrientationOfCamera);

//Print out values of matrix
printf("Camera position/orientation matrix:\n");
for(int row = 0; row < 4; row++)
{

for(int col = 0; col < 4; col ++)
{
printf("%lf ", positionAndOrientationOfCamera.at<double>(row, col));
}
printf("\n");
}

counter++;
}

// Show captured frame with overlays
imshow("captured", frame);
                                                                                                                                                          
// clean up
image.set_data(NULL, 0);

//Wait for next frame and give window a chance to render        
cv::waitKey(30);
}

return 0;
}


