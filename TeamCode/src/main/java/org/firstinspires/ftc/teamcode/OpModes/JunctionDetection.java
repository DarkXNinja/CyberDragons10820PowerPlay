package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCV.JunctionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous
public class JunctionDetection extends LinearOpMode {

    //drivetrain motors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;

    private Servo gripper;
    private Servo gripperfolder;

    public DistanceSensor rightPole;

    JunctionPipeline detector ;

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

        gripper = hardwareMap.get(Servo.class, "GrabberServo");
        gripper.setPosition(1);

        gripperfolder = hardwareMap.get(Servo.class, "GripperFolder");

        rightPole = hardwareMap.get(DistanceSensor.class, "rightPole");


        OpenCvCamera camera;
        String webcamName = "Webcam 1";


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        detector = new JunctionPipeline();

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
            telemetry.addData("Right Pole: ", " " + rightPole.getDistance(DistanceUnit.INCH));
            telemetry.update();




        }

        waitForStart();

        if (opModeIsActive()) {

            centerRobotCamera();
            //adjustRobotCaDistance() ; // use sensor
            adjustRobotCameraDistance() ; // use camera for distance


        }
        sleep(10000) ;
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

        telemetry.addData("lmscreen:", " " + lmscreen + " rmscreen: " + rmscreen);

        // displays bounding box info
        telemetry.addData("\nWidth value:", " " + detector.maxWidth + " x value: " + detector.x + " avgX: " + avgX);
        //telemetry.update();

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
                telemetry.addData("Robot is ", "moving left.");
                //telemetry.update();

                // for some reason using timer is not working
                timer.reset() ;
                moveLeft(0.25);
                while(timer.milliseconds() < 5000) {
                    sleep(100); // this is extremely important to give the OpenCV thread to execute
                    if (checkCameraWithinBounds() == 0) {
                        stopAllWheels();
                        telemetry.addData("Robot has", "reached center.");
                        telemetry.update() ;
                        break ;
                    }
                }
                stopAllWheels();

                /*
                for(int i=0;i<10;i++) {
                    moveLeft(0.5);
                    sleep(50) ;
                    stopAllWheels();
                    sleep(50) ;
                    if (checkCameraWithinBounds() == 0) {
                        telemetry.addData("Robot has", "reached center.");
                        //telemetry.update() ;
                        break ;
                    }
                    //sleep (5000) ;
                }
                 */

            } else {
                telemetry.addData("Robot is ", "moving right.");
                //telemetry.update();


                timer.reset() ;
                moveRight(0.25);
                while(timer.milliseconds() < 5000) {
                    sleep(100);// this is extremely important to give the OpenCV thread to execute
                    if (checkCameraWithinBounds() == 0) {
                        stopAllWheels();
                        telemetry.addData("Robot has", "reached center.");
                        telemetry.update() ;
                        break ;
                    }

                }
                stopAllWheels();

                /*
                for(int i=0;i<10;i++) {
                    moveRight(0.5);
                    sleep(50);
                    stopAllWheels();
                    sleep(50);
                    if (checkCameraWithinBounds() == 0) {
                        telemetry.addData("Robot has", "reached center.");
                        //telemetry.update();
                        break;
                    }
                    //sleep(5000);
                }
                 */

            }
        }

        /*
        while (timer.milliseconds() < 5000) {
            if (checkCameraWithinBounds() == 0) {
                stopAllWheels();
                telemetry.addData("Robot has", "reached center.");
                telemetry.update() ;
                break;
            }
        }
        */
        telemetry.update() ;
        stopAllWheels();
    }

    private int checkCameraDistancetoJunction() {
        int pwidth ;
        // it should be within range from the junction pole
        int minWidth = 55, maxWidth = 65;
        pwidth = detector.maxWidth ;
        if ((pwidth >= minWidth) && (pwidth <= maxWidth))
            return 0;
        else {
            if (pwidth < minWidth) // too far
                return -1 ;
            else // too close
                return 1;
        }
    }

    private int checkDistancetoJunction() {
        double pdist ;
        // it should be within range from the junction pole
        double minDist = 5.5, maxDist = 7.5;
        pdist = rightPole.getDistance(DistanceUnit.INCH) ;
        if ((pdist >= minDist) && (pdist <= maxDist))
            return 0;
        else {
            if (pdist < minDist) // too close
                return -1 ;
            else // too far
                return 1;
        }
    }

    public void adjustRobotDistance() {

        ElapsedTime timer = new ElapsedTime();
        timer.reset() ;

        int retdist = checkDistancetoJunction() ;

        if ( retdist == 0) {
            // nothing else to do
        } else {
            if (retdist < 0) {
                timer.reset();
                moveBackward(0.25);
                while (timer.milliseconds() < 5000) {
                    if (checkDistancetoJunction() == 0) {
                        stopAllWheels();
                        telemetry.addData("Robot is", "good distance from junction.");
                        telemetry.update();
                        break;
                    }

                }
                stopAllWheels();

            } else {
                timer.reset();
                moveForward(0.25);
                while (timer.milliseconds() < 5000) {
                    if (checkDistancetoJunction() == 0) {
                        stopAllWheels();
                        telemetry.addData("Robot is", "good distance from junction.");
                        telemetry.update();
                        break;
                    }

                }
                stopAllWheels();
            }
        }
    }

    public void adjustRobotCameraDistance() {

        ElapsedTime timer = new ElapsedTime();
        timer.reset() ;

        int retdist = checkCameraDistancetoJunction() ;

        if ( retdist == 0) {
            // nothing else to do
        } else {
            if (retdist < 0) {
                timer.reset();
                moveForward(0.25);
                while (timer.milliseconds() < 5000) {
                    sleep(100);// this is extremely important to give the OpenCV thread to execute
                    if (checkCameraDistancetoJunction() == 0) {
                        stopAllWheels();
                        telemetry.addData("Robot is", "good distance from junction.");
                        telemetry.update();
                        break;
                    }

                }
                stopAllWheels();

            } else {
                timer.reset();
                moveBackward(0.25);
                while (timer.milliseconds() < 5000) {
                    sleep(100);// this is extremely important to give the OpenCV thread to execute
                    if (checkCameraDistancetoJunction() == 0) {
                        stopAllWheels();
                        telemetry.addData("Robot is", "good distance from junction.");
                        telemetry.update();
                        break;
                    }

                }
                stopAllWheels();
            }
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