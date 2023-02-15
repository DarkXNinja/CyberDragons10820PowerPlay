package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SimpleJunctionPipeline extends OpenCvPipeline {

    // create instance variables for the mats that will be used
    // better to create them upfront so it reduces for each frame processing
    // and also ensures that you eliminate the possibility of forgetting to call release
    Mat mat = new Mat() ;
    Mat thresh = new Mat();
    Mat hierarchy = new Mat();

    // Low HSV and High HSV for yellow junction color (change as required)
    Scalar lowHSV = new Scalar(20, 70, 80);
    Scalar highHSV = new Scalar(32, 255, 255);

    // variables to expose results outside of this pipeline
    public int length = 0; // number of rects
    public int maxWidth = 0; // width of max rect
    public int maxHeight = 0; // height of max rect
    public int x = 0;         // x coordinate of max rect
    public int index = 0;     // array index of max rect - not needed??
    public Point topLeftCoordinate; // x,y of top left corner
    public Point bottomRightCoordinate;  // x,y of bottom right corner


    @Override
    public Mat processFrame(Mat input) {

        //turn input to hsv; input -> mat
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // Get a black and white image of yellow objects
        // pixels that are within filter bounds show as white, and those outside range are black
        // mat -> thresh
        Core.inRange(mat, lowHSV, highHSV, thresh);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        //find contours, input thresh because it has hard edges
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //bounding boxes
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));

            // just color in the bounding rectangle so it is easy to see - can be commented out also
            Imgproc.rectangle(thresh, boundRect[i], new Scalar(255, 0, 0));

        }

        length = boundRect.length;
        if (length != 0) {
            // finds largest box(width) value of all boxes created
            maxWidth = 0;
            index = 0;
            for (int i = 0; i < length; i++) {
                if (boundRect[i].width > maxWidth) {
                    index = i;
                    maxWidth = boundRect[i].width;
                }
            }
            topLeftCoordinate = boundRect[index].tl();
            bottomRightCoordinate = boundRect[index].br();
            x = boundRect[index].x;
            maxWidth = boundRect[index].width;
            maxHeight = boundRect[index].height;
        }

         

        return thresh;
    }

}
