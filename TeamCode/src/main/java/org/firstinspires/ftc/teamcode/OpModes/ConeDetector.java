package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCV.ConePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class ConeDetector extends LinearOpMode {
    //drivetrain motors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;

    public DistanceSensor rightPole;

    ConePipeline detector ;

    public int avgX;
    public int screenWidth = 320;

    @Override
    public void runOpMode() throws InterruptedException {

        // setting up drive train
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        OpenCvCamera camera;
        String webcamName = "Webcam 1";


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        detector = new ConePipeline();

        camera.setPipeline(detector);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                camera.startStreaming(screenWidth,240, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {



            telemetry.addData("number of bounding boxes: ", detector.length);

            // displays bounding box info
            telemetry.addData("\nWidth value: ", detector.maxWidth);
            telemetry.addData("Height value: ", detector.maxHeight);
            telemetry.addData("\nTop-Left Corner: ", detector.topLeftCoordinate);
            telemetry.addData("Bottom-Right Corner: ", detector.bottomRightCoordinate);
            telemetry.update();




        }

        waitForStart();

        if (opModeIsActive()) {

            centerRobotCamera();



        }
    }

    // function for checking if camera is within bounds
    // 0 means it is centered
    // <1 means it is to the left
    // >1 means it is to the right
    private int checkCameraWithinBounds () {
        int midScreen = screenWidth/2 ;
        int sTolerance = 10 ; // this is the number of pixels
        int lmscreen = midScreen - sTolerance ;
        int rmscreen = midScreen + sTolerance ;

        avgX = (detector.maxWidth/2) + detector.x;

        // if it is mostly centered then
        if ((avgX >= lmscreen) && (avgX <= rmscreen)) {
            return 0 ;
        } else {
            if (avgX < lmscreen)
                return (avgX - lmscreen) ;
            else
                return (avgX - rmscreen) ;
        }
    }

    public void centerRobotCamera() {

        ElapsedTime timer = new ElapsedTime();
        timer.reset() ;

        int rpos = checkCameraWithinBounds() ;
        if (rpos == 0) {
            telemetry.addData("Robot is ", "already centered.");
            telemetry.update();
            return;
        }
        else {
            if (rpos < 0) {
                moveRight(0.5);
                telemetry.addData("Robot is ", "moving right.");
            } else {
                moveLeft(0.5);
                telemetry.addData("Robot is ", "moving left.");
            }
        }

        while (timer.milliseconds() < 5000) {
            if (checkCameraWithinBounds() == 0) {
                stopAllWheels();
                telemetry.addData("Robot has", "reached center.");
                telemetry.update() ;
                break;
            }
        }

        stopAllWheels();
        telemetry.addData("Robot has", "reached time limit.");
        telemetry.update() ;

    }

    void moveBackward(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);



    }

    void moveForward(double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

    }

    void moveLeft(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

    }

    void stopAllWheels() {

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);

    }

    void moveRight(double speed)  {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

    }

}