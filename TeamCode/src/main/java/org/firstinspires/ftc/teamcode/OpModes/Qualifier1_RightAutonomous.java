package org.firstinspires.ftc.teamcode.OpModes;

import android.text.method.Touch;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.RoadRunnerConfiguration.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunnerConfiguration.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Qualifier1_RightAutonomous extends LinearOpMode {


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
    private CRServo gripperfolder;

    private DistanceSensor gripperHeight;
    private DistanceSensor rightPole;
    private DistanceSensor leftPole;

    SampleMecanumDrive drive ;

    // these are short trajectories that can be used for adjustment
    Trajectory strafeLeft1, strafeRight1, forward1, back1 ;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        //Starting the robot at the bottom left (blue auto)
        Pose2d startPose = new Pose2d(-36, 72, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        double slowerVel = 24.0;

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

        Trajectory autoTrajectory1 = drive.trajectoryBuilder(startPose)
                .forward(3.0)
                .build();

        Trajectory autoTrajectory2 = drive.trajectoryBuilder(autoTrajectory1.end())
                .strafeLeft(26.0)
                .build();

        Trajectory autoTrajectory3 = drive.trajectoryBuilder(autoTrajectory2.end())
                .lineToLinearHeading(new Pose2d(-12, 28, Math.toRadians(360)),
                        SampleMecanumDrive.getVelocityConstraint(slowerVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory autoTrajectory4 = drive.trajectoryBuilder(autoTrajectory3.end())
                .forward(3.0)
                .build();

        Trajectory autoTrajectory5 = drive.trajectoryBuilder(autoTrajectory4.end())
                .back(3.0)
                .build();

        Trajectory autoTrajectory6 = drive.trajectoryBuilder(autoTrajectory5.end())
                .strafeLeft(16.0)
                .build();


        // below few trajectories are used for the adjustRobotPositionToJunction
        strafeLeft1 = drive.trajectoryBuilder(autoTrajectory5.end())
                .strafeLeft(1.0)
                .build();

        strafeRight1 = drive.trajectoryBuilder(autoTrajectory5.end())
                .strafeRight(1.0)
                .build();

        forward1 = drive.trajectoryBuilder(autoTrajectory5.end())
                .forward(1.0)
                .build();

        back1 = drive.trajectoryBuilder(autoTrajectory5.end())
                .back(1.0)
                .build();


        drive.followTrajectory(
                // moves forward
                autoTrajectory1);

        drive.followTrajectory(
                // strafes left
                autoTrajectory2);

        moveLiftToPositionAsync(-6600);

        drive.followTrajectory(
                // goes to high junction and drops cone
                autoTrajectory3);

        checkLiftInPositionAsync(-6600);

        /*
        drive.followTrajectory(
                // goes forward
                autoTrajectory4);
        */

        Thread.sleep(500);

        gripperfolder.setPower(1.0);
        Thread.sleep(1750);
        gripperfolder.setPower(0);

        //open gripper
        gripper.setPosition(0);

        Thread.sleep(250);

        /*
        drive.followTrajectory(
                // goes backward to clear junction
                autoTrajectory5);
        */

        Thread.sleep(50);
        drive.followTrajectory(
                // strafes left to parking zones
                autoTrajectory6);


        gripperfolder.setPower(-1.0);
        Thread.sleep(1750);
        gripperfolder.setPower(0);

        // Actually do something useful
        if (tagOfInterest == null || tagOfInterest.id == MIDDLE) {

            //trajectory
            Trajectory zone2 = drive.trajectoryBuilder(autoTrajectory6.end())
                    .back(24.0)
                    .build();

            drive.followTrajectory(zone2);

        } else if (tagOfInterest.id == RIGHT) {

            //trajectory
            Trajectory zone3 = drive.trajectoryBuilder(autoTrajectory6.end())
                    .back(48.0)
                    .build();

            drive.followTrajectory(zone3);

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
        lift1 = hardwareMap.get(DcMotorEx.class,"lift1");
        lift2 = hardwareMap.get(DcMotorEx.class,"lift2");

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gripper = hardwareMap.get(Servo.class, "GrabberServo");
        gripper.setPosition(1);

        gripperfolder = hardwareMap.get(CRServo.class, "GripperFolder");

        gripperHeight = hardwareMap.get(DistanceSensor.class, "gripperHeight");
        rightPole = hardwareMap.get(DistanceSensor.class, "rightPole");
        leftPole = hardwareMap.get(DistanceSensor.class, "leftPole");

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

    public boolean adjustRobotPositionToJunction () {
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

                for (int i=0;i<12;i++) {
                    drive.followTrajectory(
                            // strafeLeft1 inch
                            strafeLeft1);
                    rightDist = rightPole.getDistance(DistanceUnit.INCH) ;
                    leftDist = leftPole.getDistance(DistanceUnit.INCH) ;
                    if ((rightDist < maxPD) && (leftDist < maxPD)) {
                        angleGood = true ;
                        break;
                    }
                }
                if (!angleGood) {
                    // we tried a lot and didnt succeed, so we give up
                    return false;
                }
            } else {
                if ((leftDist >= maxPD) && (rightDist <= maxPD)) {
                    for (int i=0;i<12;i++) {
                        drive.followTrajectory(
                                // strafeLeft1 inch
                                strafeRight1);
                        rightDist = rightPole.getDistance(DistanceUnit.INCH) ;
                        leftDist = leftPole.getDistance(DistanceUnit.INCH) ;
                        if ((rightDist < maxPD) && (leftDist < maxPD)) {
                            angleGood = true ;
                            break;
                        }
                    }
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

        // now adjust for distance from the junction
        // if it is too close then it wont work properly because of the grabber folder
        if ((rightDist < maxPD) || (leftDist < maxPD)) {
            for (int i=0;i<12;i++) {
                drive.followTrajectory(
                        // strafeLeft1 inch
                        back1);
                rightDist = rightPole.getDistance(DistanceUnit.INCH) ;
                leftDist = leftPole.getDistance(DistanceUnit.INCH) ;
                if ((rightDist > maxPD) || (leftDist > maxPD)) {
                    distGood = true ;
                    break;
                }
            }
            if (!distGood)
                return false ;

        }
        return true ;
    }
}
