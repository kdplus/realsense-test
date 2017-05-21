#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void f_gray2color(Mat gray_mat,Mat color_mat)
{
    uchar* p=nullptr;
    Vec3b* q=nullptr;
    int height=gray_mat.rows;
    int width=gray_mat.cols;
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            p=gray_mat.ptr<uchar>(i);
            q=color_mat.ptr<Vec3b>(i);
            if(p[j]==0)
            {q[j][0]=0;q[j][1]=0;q[j][2]=0;}
            else
            {
                q[j][0]=255-p[j];
                q[j][1]=128-abs(128-p[j]);
                q[j][2]=p[j];
            }
        }
    }
    return;
}


float getDistance(rs::device* dev, int x, int y){
    uint16_t *depthImage = (uint16_t *) dev->get_frame_data(rs::stream::depth);
    float scale = dev->get_depth_scale();
    rs::intrinsics depthIntrin = dev->get_stream_intrinsics(rs::stream::depth);
    uint16_t depthValue = depthImage[y * depthIntrin.width + x];
    float depthInMeters = depthValue * scale;
    return depthInMeters;
}


int main()
{
    rs::context ctx;
    rs::device * dev = ctx.get_device(0);

    // Configure Infrared stream to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30);

    // We must also configure depth stream in order to IR stream run properly
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);

    // Start streaming
    dev->start();


while(true){
    // Camera warmup - Dropped frames to allow stabilization
    for(int i = 0; i < 2; i++)
        dev->wait_for_frames();
    
    // Creating OpenCV matrix from IR image
    Mat ir(Size(640, 480), CV_16UC1, (void*)dev->get_frame_data(rs::stream::depth), Mat::AUTO_STEP);

    // Apply Histogram Equalization
    //equalizeHist( ir, ir );
    //applyColorMap(ir, ir, COLORMAP_JET);

    Mat output;
    ir.convertTo(output, CV_8UC1, 1);
    Mat color(480, 640, CV_8UC3);
    f_gray2color(output, color);
    // Display the image in GUI
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    cout << getDistance(dev, 200, 200) << endl;
    imshow("Display Image", ir);
    waitKey(1);
}

    return 0;
}
