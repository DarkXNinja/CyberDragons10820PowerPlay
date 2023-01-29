package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCV.JunctionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class JunctionDetection extends LinearOpMode {

    //drivetrain motors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;

    public DistanceSensor rightPole;

    public double avgX;
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

        JunctionPipeline detector = new JunctionPipeline();

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

        avgX = (detector.maxWidth/2) + detector.x;

        if (opModeIsActive()) {

            centerRobotCamera();



        }
    }

    public void centerRobotCamera() {

        if (avgX < (screenWidth/2)) {

            moveRight(0.5);
            while (avgX < (screenWidth/2)) {

            }
            stopAllWheels();

        } else if (avgX > (screenWidth/2)) {

            moveLeft(0.5);
            while (avgX > (screenWidth/2)) {

            }
            stopAllWheels();

        } else {

            telemetry.addData("Robot is ", "already centered.");
            telemetry.update();

        }


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