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

//for dashboard
/*@Config*/
public class JunctionPipeline extends OpenCvPipeline {

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;

    // create instance variables for the mats that need to be used
    // do not use local variables; reduces CPU load
    // and also ensures that you eliminate the possibility of forgetting to call release
    Mat mat = new Mat() ;
    Mat thresh = new Mat();
    Mat masked = new Mat();
    Mat scaledMask = new Mat();
    Mat scaledThresh = new Mat();
    Mat finalMask = new Mat();
    Mat hierarchy = new Mat();
    Mat edges = new Mat();

    public int length = 0;
    public int maxWidth = 0;
    public int maxHeight = 0;
    public int x = 0;
    public int index = 0;
    public Point topLeftCoordinate;
    public Point bottomRightCoordinate;


    public JunctionPipeline() {
        frameList = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {

        //turn input to hsv; input -> mat
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

        // Get a black and white image of yellow objects
        // pixels that are within filter bounds have value 255, and those outside range are black
        // mat -> thresh
        Core.inRange(mat, lowHSV, highHSV, thresh);

        //color the white portion of thresh in with HSV from mat
        //(mat, thresh) -> masked
        Core.bitwise_and(mat, mat, masked, thresh);

        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        //scale the average saturation to 150
        // masked -> scaledMask
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        // scaledMask -> scaledThresh
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        // scaledThresh -> finalMask
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        //detect edges(only useful for showing result)(you can delete)
        // scaledThresh -> edges
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        //find contours, input scaledThresh because it has hard edges
        // scaledThresh -> hierarchy
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //bounding boxes
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));

        }

        for (int i = 0; i != boundRect.length; i++) {

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(scaledMask, boundRect[i], new Scalar(0, 255, 255));

        }

        length = boundRect.length;

        if (boundRect.length != 0) {

            // finds largest box(width) value of all boxes created
            maxWidth = 0;
            index = 0;
            for (int i = 0; i < boundRect.length; i++) {

                if (boundRect[i].width >  maxWidth) {

                    index = i;
                    maxWidth = boundRect[i].width;

                }
            }


            topLeftCoordinate = boundRect[index].tl();
            bottomRightCoordinate = boundRect[index].br();
            x = boundRect[index].x;

            maxWidth = boundRect[index].width;
            maxHeight = boundRect[index].height;

            //topLeftCoordinate = "(" + boundRect[index].x + ", " + boundRect[index].y + ")";
            //bottomRightCoordinate = "(" + (boundRect[index].x + boundRect[index].width) + ", " + (boundRect[index].y + boundRect[index].height) + ")";


        }


        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }


        //release all the data
        //input.release();
        //scaledThresh.copyTo(input);
        //scaledThresh.release();
        //scaledMask.release();
        //mat.release();
        //masked.release();
        //edges.release();
        //thresh.release();
        //finalMask.release();
        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)

        return scaledMask ;
    }


}