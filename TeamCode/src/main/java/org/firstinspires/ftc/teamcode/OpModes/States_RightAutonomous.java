package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.JunctionPipeline;
import org.firstinspires.ftc.teamcode.OpenCV.SimpleJunctionPipeline;
import org.firstinspires.ftc.teamcode.RoadRunnerConfiguration.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunnerConfiguration.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class States_RightAutonomous extends LinearOpMode {


    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    SimpleJunctionPipeline detector;

    static final double FEET_PER_METER = 3.28084;

    final int ttolerance = 25;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    ElapsedTime timer = new ElapsedTime();

    DcMotorEx lift1;
    DcMotorEx lift2;

    //drivetrain motors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;

    private Servo gripper;
    private Servo gripperfolder;

    private DistanceSensor gripperHeight;
    private DistanceSensor rightPole;
    private DistanceSensor junctionSensor;


    SampleMecanumDrive drive;

    ElapsedTime centeringTimer = new ElapsedTime();

    public int avgX;
    public int screenWidth = 320;
    /*
    5 - 650
    4 - 450
    3 - 275
    2 - 125
    1 - 0
     */


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        //Starting the robot at the bottom left (blue auto)
        Pose2d startPose = new Pose2d(-36, 72, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        double slowerVel = 24.0;
        double slowerVel2 = 5.0;

        initializeRobot();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // init loop
        while (!isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {

                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);

                } else {

                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {

                        telemetry.addLine("(The tag has never been seen)");

                    } else {

                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);

                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {

                    telemetry.addLine("(The tag has never been seen)");

                } else {

                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);

        }
        /* Update the telemetry */
        if (tagOfInterest != null) {

            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();

        } else {

            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        // create all trajectories before start is pressed ; reduces time spent in autonomous
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(3.0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(26.0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-11, 26, Math.toRadians(360)),
                        SampleMecanumDrive.getVelocityConstraint(slowerVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0,0, Math.toRadians(-180))))
                .strafeLeft(18.0)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(-55, 12, Math.toRadians(180)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .back(4.0,
                        SampleMecanumDrive.getVelocityConstraint(slowerVel2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .back(6.0)
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(180)))
                .strafeRight(6.0)
                .build();

        if (isStopRequested()) return;

        camera.pauseViewport(); // this reduces CPU/battery load

        drive.followTrajectory(
                // moves forward
                traj1);

        drive.followTrajectory(
                // strafes left
                traj2);

        moveLiftToPositionAsync(-3500);

        drive.followTrajectory(
                // goes to high junction and drops cone
                traj3);

        checkLiftInPositionAsync(-3500);

        Thread.sleep(50);

        moveForwardRoadRunner(0.25);
        centeringTimer.reset();
        while ((junctionSensor.getDistance(DistanceUnit.INCH) > 2) && (centeringTimer.milliseconds() < 3000)) {
            // keep moving forward until it get close to the bare junction until a certain time
        }
        stopAllWheelsRoadRunner();

        moveBackwardRoadRunner(0.25);
        centeringTimer.reset();
        while ((junctionSensor.getDistance(DistanceUnit.INCH) < 9) && ((centeringTimer.milliseconds() < 5000))) {
            // keep moving back till we are a distance away from the junction, but only until a certain time
        }
        stopAllWheelsRoadRunner();

        Thread.sleep(250);

        gripperfolder.setPosition(1.0);
        Thread.sleep(1000);
        gripper.setPosition(0);


        gripperfolder.setPosition(0.0);
        drive.turn(Math.toRadians(-180));
        moveLiftToPositionAsync(-500);
        checkLiftInPositionAsync(-500);

        drive.followTrajectory(
                // strafes left
                traj4);

        gripperfolder.setPosition(1.0);
        drive.followTrajectory(
                //goes to starter stack #1
                traj5);

        // cone 1
        gripper.setPosition(1.0);

        gripper.setPosition(1.0);
        Thread.sleep(250);

        moveLiftToPositionAsync(-1200);
        drive.followTrajectory(
                //takes cone #1
                traj6);

        checkLiftInPositionAsync(-1200);

        sleep(50);

        moveLiftToPositionAsync(-3500);
        gripperfolder.setPosition(0.0);

        drive.followTrajectory(
                //back some more
                traj7);

        drive.turn(Math.toRadians(165));

        checkLiftInPositionAsync(-3500);

        moveForwardRoadRunner(0.25);
        centeringTimer.reset();
        while ((junctionSensor.getDistance(DistanceUnit.INCH) > 2) && (centeringTimer.milliseconds() < 3000)) {
            // keep moving forward until it get close to the bare junction until a certain time
        }
        stopAllWheelsRoadRunner();

        moveBackwardRoadRunner(0.25);
        centeringTimer.reset();
        while ((junctionSensor.getDistance(DistanceUnit.INCH) < 9) && ((centeringTimer.milliseconds() < 5000))) {
            // keep moving back till we are a distance away from the junction, but only until a certain time
        }
        stopAllWheelsRoadRunner();

        Thread.sleep(250);

        gripperfolder.setPosition(1.0);
        Thread.sleep(1000);
        gripper.setPosition(0);

        drive.turn(Math.toRadians(-165));

        drive.followTrajectory(
                //strafes right to parking zones
                traj8);

        // Actually do something useful
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {


            Trajectory zone3 = drive.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(180)))
                    .forward(12.0)
                    .build();

            drive.followTrajectory(zone3);



        } else if (tagOfInterest.id == RIGHT) {


            Trajectory zone1 = drive.trajectoryBuilder(new Pose2d(0,0, Math.toRadians(180)))
                    .back(12.0)
                    .build();

            drive.followTrajectory(zone1);



        }


    }


    public int detect() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        timer.reset();
        while (timer.milliseconds() < 2000) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {

                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);

                } else {

                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {

                        telemetry.addLine("(The tag has never been seen)");

                    } else {

                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);

                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {

                    telemetry.addLine("(The tag has never been seen)");

                } else {

                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);

        }


        return tagOfInterest.id;

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


    void initializeRobot()
    {

        /*
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

         */

        lift1 = hardwareMap.get(DcMotorEx.class,"lift1");
        lift2 = hardwareMap.get(DcMotorEx.class,"lift2");

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //reversing lift 2
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        gripper = hardwareMap.get(Servo.class, "GrabberServo");
        gripper.setPosition(1);

        gripperfolder = hardwareMap.get(Servo.class, "GripperFolder");
        gripperfolder.setPosition(0.0);

        gripperHeight = hardwareMap.get(DistanceSensor.class, "gripperHeight");
        rightPole = hardwareMap.get(DistanceSensor.class, "rightPole");
        junctionSensor = hardwareMap.get(DistanceSensor.class, "junctionSensor");


        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    void moveLiftToPosition(int pos) {
        int tposition = pos;
        int thigher = tposition - ttolerance;
        int tlower = tposition + ttolerance;

        lift1.setTargetPosition(tposition);
        lift1.setTargetPositionTolerance(ttolerance);
        lift2.setTargetPosition(tposition);
        lift2.setTargetPositionTolerance(ttolerance);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(1.0);
        lift2.setPower(1.0);

        while (true) {
            int lift1position = lift1.getCurrentPosition();
            int lift2position = lift2.getCurrentPosition();

            if ( ((lift1position < tlower) && (lift1position > thigher)) ||
                    ((lift2position < tlower) && (lift2position > thigher)) ) {

                lift1.setPower(0.0);
                lift2.setPower(0.0);
                break;
            }
        }
    }

    void moveLiftToPositionAsync(int pos) {
        int tposition = pos;
        int thigher = tposition - ttolerance;
        int tlower = tposition + ttolerance;

        lift1.setTargetPosition(tposition);
        lift1.setTargetPositionTolerance(ttolerance);
        lift2.setTargetPosition(tposition);
        lift2.setTargetPositionTolerance(ttolerance);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(1.0);
        lift2.setPower(1.0);

        // make sure you call checkListInPosition to make sure lift is in position
    }

    void checkLiftInPositionAsync(int pos) {
        int tposition = pos;
        int thigher = tposition - ttolerance;
        int tlower = tposition + ttolerance;

        while (true) {
            int lift1position = lift1.getCurrentPosition();
            int lift2position = lift2.getCurrentPosition();

            if (((lift1position < tlower) && (lift1position > thigher)) ||
                    ((lift2position < tlower) && (lift2position > thigher))) {

                lift1.setPower(0.0);
                lift2.setPower(0.0);
                break;
            }
        }
    }

    void initializeOpenCV() {

        detector = new SimpleJunctionPipeline();

        this.camera.setPipeline(detector);

    }

    void initializeDrive() {

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

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    void initializeRoadRunner(Pose2d pose) {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(pose);


    }

    void placeCone() {

        centerRobotCamera();
        sleep(5000);
        // if centered successfully; then stop streaming
        camera.stopStreaming();

        adjustRobotDistance() ; // distance use sensor
        //adjustRobotCameraDistance() ; // use camera for distance
        sleep(5000);

        // down and open gripper
        downGripper();
        sleep(500) ;
        openGripper();

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
                moveLeftRoadRunner(0.25);
                while(timer.milliseconds() < 5000) {
                    sleep(100); // this is extremely important to give the OpenCV thread to execute
                    // need to call function
                    if (checkCameraWithinBounds() == 0) {
                        stopAllWheelsRoadRunner();
                        telemetry.addData("Robot has", "reached center.");
                        telemetry.update() ;
                        break ;
                    }
                }
                stopAllWheelsRoadRunner();

            } else {
                telemetry.addData("Robot is ", "moving right.");
                //telemetry.update();


                timer.reset() ;
                moveRightRoadRunner(0.25);
                while(timer.milliseconds() < 5000) {
                    sleep(100);// this is extremely important to give the OpenCV thread to execute
                    if (checkCameraWithinBounds() == 0) {
                        stopAllWheelsRoadRunner();
                        telemetry.addData("Robot has", "reached center.");
                        telemetry.update() ;
                        break ;
                    }

                }
                stopAllWheelsRoadRunner();

            }
        }

        telemetry.update() ;
        stopAllWheelsRoadRunner();
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
        double minDist = 10.5, maxDist = 11.5;
        pdist = junctionSensor.getDistance(DistanceUnit.INCH) ;
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
                moveBackwardRoadRunner(0.2);
                while (timer.milliseconds() < 5000) {
                    if (checkDistancetoJunction() == 0) {
                        stopAllWheelsRoadRunner();
                        telemetry.addData("Robot is", "good distance from junction.");
                        telemetry.update();
                        break;
                    }

                }
                stopAllWheelsRoadRunner();

            } else {
                timer.reset();
                moveForwardRoadRunner(0.2);
                while (timer.milliseconds() < 5000) {
                    if (checkDistancetoJunction() == 0) {
                        stopAllWheelsRoadRunner();
                        telemetry.addData("Robot is", "good distance from junction.");
                        telemetry.update();
                        break;
                    }

                }
                stopAllWheelsRoadRunner();
            }
        }
    }

    private void openGripper() {
        gripper.setPosition(0); // open gripper
    }
    private void closeGripper() {
        gripper.setPosition(1); // close gripper
    }
    private void upGripper() {
        gripperfolder.setPosition(0); // up gripper
    }
    private void downGripper() {
        gripperfolder.setPosition(1); // down gripper
    }

    void moveBackwardRoadRunner(double speed) {

        drive.setMotorPowers(-speed, -speed, -speed, -speed);

    }


    void moveForwardRoadRunner(double speed) {

        drive.setMotorPowers(speed, speed, speed, speed);

    }

    void moveRightRoadRunner(double speed) {

        drive.setMotorPowers(speed, -speed, speed, -speed);
    }


    void moveLeftRoadRunner(double speed)  {

        drive.setMotorPowers(-speed, speed, -speed, speed);

    }

    void stopAllWheelsRoadRunner() {

        drive.setMotorPowers(0, 0, 0, 0);


    }



}
