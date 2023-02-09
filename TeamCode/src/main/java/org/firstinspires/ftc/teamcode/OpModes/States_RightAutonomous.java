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
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
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
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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


    private Servo gripper;
    private Servo gripperfolder;

    private DistanceSensor gripperHeight;
    private DistanceSensor rightPole;

    SampleMecanumDrive drive ;

    ElapsedTime centeringTimer = new ElapsedTime();

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


        if (isStopRequested()) return;

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
                .strafeLeft(16.0)
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

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end().plus(new Pose2d(0,0, Math.toRadians(-135))))
                .forward(2.0,
                        SampleMecanumDrive.getVelocityConstraint(slowerVel2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .back(2.0,
                        SampleMecanumDrive.getVelocityConstraint(slowerVel2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj6.end().plus(new Pose2d(0,0, Math.toRadians(135))))
                .forward(2.0,
                        SampleMecanumDrive.getVelocityConstraint(slowerVel2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        drive.followTrajectory(
                // moves forward
                traj1);

        drive.followTrajectory(
                // strafes left
                traj2);

        moveLiftToPositionAsync(-3800);

        drive.followTrajectory(
                // goes to high junction and drops cone
                traj3);

        checkLiftInPositionAsync(-3800);

        Thread.sleep(50);

        //open gripper
        gripper.setPosition(0);

        drive.turn(Math.toRadians(-180));

        moveLiftToPositionAsync(-650);

        drive.followTrajectory(
                // strafes left
                traj4);

        drive.followTrajectory(
                //goes to starter stack #1
                traj5);

        checkLiftInPositionAsync(-650);

        // cone 1
        gripper.setPosition(1.0);
        Thread.sleep(250);

        moveLiftToPositionAsync(-1200);
        drive.followTrajectory(
                //takes cone #1
                traj6);

        checkLiftInPositionAsync(-1200);

        sleep(50);

        moveLiftToPositionAsync(-1800);

        drive.turn(Math.toRadians(-135));

        drive.followTrajectory(
                //forward a little
                traj7);

        gripper.setPosition(0);

        Thread.sleep(250);

        drive.followTrajectory(
                //backward a little
                traj8);

        drive.turn(Math.toRadians(135));

        // cone 2
        moveLiftToPositionAsync(-450);

        drive.followTrajectory(
                //forward a little
                traj9);

        checkLiftInPositionAsync(-450);

        gripper.setPosition(1.0);

        gripper.setPosition(1.0);
        Thread.sleep(250);

        moveLiftToPositionAsync(-1000);
        drive.followTrajectory(
                //takes cone #2
                traj6);

        checkLiftInPositionAsync(-1000);

        sleep(50);

        moveLiftToPositionAsync(-1800);

        drive.turn(Math.toRadians(-135));

        drive.followTrajectory(
                //forward a little
                traj7);

        gripper.setPosition(0);

        Thread.sleep(250);

        drive.followTrajectory(
                //backward a little
                traj8);

        drive.turn(Math.toRadians(135));




        // Actually do something useful
        if (tagOfInterest == null || tagOfInterest.id == MIDDLE) {

            /*
            //trajectory
            Trajectory zone2 = drive.trajectoryBuilder()
                    .back(24.0)
                    .build();

            drive.followTrajectory(zone2);


             */
        } else if (tagOfInterest.id == RIGHT) {
            /*
            //trajectory
            Trajectory zone3 = drive.trajectoryBuilder()
                    .back(48.0)
                    .build();

            drive.followTrajectory(zone3);


             */
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
        gripperfolder.setPosition(1.0);

        gripperHeight = hardwareMap.get(DistanceSensor.class, "gripperHeight");
        rightPole = hardwareMap.get(DistanceSensor.class, "rightPole");

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
/*
    void moveBackward(double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);



    }

    void moveForward(double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

    }

    void moveLeft(double speed) throws InterruptedException {
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

    void moveRight(double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);


    }

    public boolean adjustRobotPositionToJunction() throws InterruptedException {

        gripper.setPosition(1);


        double rightDist = rightPole.getDistance(DistanceUnit.INCH) ;
        double leftDist = leftPole.getDistance(DistanceUnit.INCH) ;

        double maxPD = 12.0 ;

        boolean angleGood = false;
        boolean distGood = false ;

        if ((rightDist < maxPD) && (leftDist < maxPD)) {
            // then it is at a good angle relative to the junction
            // we are good to go on angle
        } else {
            if ((rightDist >= maxPD) && (leftDist <= maxPD)) {

                centeringTimer.reset();
                while (centeringTimer.milliseconds() < 750) {

                    moveLeft(0.25);
                    rightDist = rightPole.getDistance(DistanceUnit.INCH) ;
                    leftDist = leftPole.getDistance(DistanceUnit.INCH) ;
                    if ((rightDist < maxPD) && (leftDist < maxPD)) {
                        angleGood = true ;
                        break;
                    }


                }
                stopAllWheels();

                if (!angleGood) {
                    // we tried a lot and didnt succeed, so we give up
                    return false;
                }
            } else {
                if ((leftDist >= maxPD) && (rightDist <= maxPD)) {

                    centeringTimer.reset();
                    while (centeringTimer.milliseconds() < 750) {

                        moveRight( 0.25);

                        rightDist = rightPole.getDistance(DistanceUnit.INCH) ;
                        leftDist = leftPole.getDistance(DistanceUnit.INCH) ;
                        if ((rightDist < maxPD) && (leftDist < maxPD)) {
                            angleGood = true ;
                            break;
                        }

                    }

                    stopAllWheels();

                    if (!angleGood)
                        return false ;

                } else {
                    // Neither of the sensors have provided reasonable values
                    // so abort
                    return false ;
                }
            }
        } // else for both being not close

        // if it has reached here, then we know that it is at a good angle relative to junction
        telemetry.addLine("Height: " + gripperHeight.getDistance(DistanceUnit.INCH));
        telemetry.addLine("pole distance left: " + leftPole.getDistance(DistanceUnit.INCH));
        telemetry.addLine("pole distance right: " + rightPole.getDistance(DistanceUnit.INCH));

        telemetry.addLine("lift1 encoder: " + lift1.getCurrentPosition());
        telemetry.addLine("lift2 encoder: " + lift2.getCurrentPosition());
        telemetry.update();
        Thread.sleep(1000);

        // now adjust for distance from the junction
        // if it is too close then it wont work properly because of the grabber folder
        if ((rightDist < maxPD) && (leftDist < maxPD)) {

            centeringTimer.reset();
            while (centeringTimer.milliseconds() < 750) {

                moveBackward(0.25);

                rightDist = rightPole.getDistance(DistanceUnit.INCH) ;
                leftDist = leftPole.getDistance(DistanceUnit.INCH) ;
                if ((rightDist > 9) && (leftDist > 9)) {
                    stopAllWheels();
                    distGood = true ;
                    break;
                }

            }


            if (!distGood)
                return false ;

        }

        return true ;


    }

    */

}