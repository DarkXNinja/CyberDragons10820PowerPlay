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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
public class Qualifier1_LeftAutonomous extends LinearOpMode {


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
    private DistanceSensor leftPole;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, 72, Math.toRadians(270));
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
                .strafeRight(26.0)
                .build();

        Trajectory autoTrajectory3 = drive.trajectoryBuilder(autoTrajectory2.end())
                .lineToLinearHeading(new Pose2d(14, 27, Math.toRadians(180)),
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
                .strafeRight(16.0)
                .build();

        drive.followTrajectory(
                // moves forward
                autoTrajectory1);

        drive.followTrajectory(
                // strafes left
                autoTrajectory2);

        moveLiftToPositionAsync(-3000);

        drive.followTrajectory(
                // goes to high junction and drops cone
                autoTrajectory3);

        checkLiftInPositionAsync(-3000);

        /*
        drive.followTrajectory(
                // goes forward
                autoTrajectory4);
        */

        Thread.sleep(500);

        // gripper down

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
                // strafes right to parking zones
                autoTrajectory6);

        // gripper up

        // Actually do something useful
        if (tagOfInterest == null || tagOfInterest.id == MIDDLE) {

            //trajectory
            Trajectory zone2 = drive.trajectoryBuilder(autoTrajectory6.end())
                    .back(20.0)
                    .build();

            drive.followTrajectory(zone2);

        } else if (tagOfInterest.id == LEFT) {

            //trajectory
            Trajectory zone1 = drive.trajectoryBuilder(autoTrajectory6.end())
                    .back(48.0)
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

}
