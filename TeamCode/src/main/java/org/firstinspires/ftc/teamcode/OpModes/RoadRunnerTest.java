package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class RoadRunnerTest extends LinearOpMode {


  OpenCvCamera camera;
  AprilTagDetectionPipeline aprilTagDetectionPipeline;

  static final double FEET_PER_METER = 3.28084;

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





  @Override
  public void runOpMode() throws InterruptedException {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    //Starting the robot at the bottom left (blue auto)
    Pose2d startPose = new Pose2d(-36, 72, Math.toRadians(270));
    drive.setPoseEstimate(startPose);

    double slowerVel = 24.0;

    waitForStart();

    if (isStopRequested()) return;


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
      /* Update the telemetry */
      if (tagOfInterest != null) {

        telemetry.addLine("Tag snapshot:\n");
        tagToTelemetry(tagOfInterest);
        telemetry.update();

      } else {

        telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
        telemetry.update();
      }

      //Makes the path for the robot to follow
      //.splineTo(new Vector2d(xCord, yCord), rotation)
      Trajectory autoTrajectory1 = drive.trajectoryBuilder(new Pose2d(-36, 72, Math.toRadians(270)))
              //Step 1: Drop off cone at low junction
              //.forward(24.0)
              .lineToLinearHeading(new Pose2d(-40, 54, Math.toRadians(180)))
              .addDisplacementMarker(() -> {
                // Run your action in here!
                //grabberServo.setPower(1.0);
              })
              .build();

      Trajectory autoTrajectory2 = drive.trajectoryBuilder(autoTrajectory1.end())
              //.lineToLinearHeading(new Pose2d(-34, 57, Math.toRadians(270)))
              .back(6.0)
              .build();

      Trajectory autoTrajectory5 = drive.trajectoryBuilder(autoTrajectory2.end())
              .strafeRight(3.0)
              .build();

      Trajectory autoTrajectory3 = drive.trajectoryBuilder(autoTrajectory5.end().plus(new Pose2d(0,0,Math.toRadians(90))))
              //.lineToLinearHeading(new Pose2d(-34,58, Math.toRadians(270)))
              .forward(9.0)
              .addDisplacementMarker(() -> {
                // Run your action in here!
                //grabberServo.setPower(-1.0);
              })
              .build();

      Trajectory autoTrajectory4 = drive.trajectoryBuilder(autoTrajectory3.end())
              .lineToLinearHeading(new Pose2d(-36, 28, Math.toRadians(360)),
                      SampleMecanumDrive.getVelocityConstraint(slowerVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
              )
              .addDisplacementMarker(() -> {
                // Run your action in here!
                //grabberServo.setPower(1.0);
              })
              .build();

      Trajectory autoTrajectory6 = drive.trajectoryBuilder(autoTrajectory4.end().plus(new Pose2d(0,0,Math.toRadians(180))))
              .strafeLeft(8.0)
              .build();

      Trajectory autoTrajectory7 = drive.trajectoryBuilder(autoTrajectory6.end())
              .lineTo(new Vector2d(-66, 12))
              .addTemporalMarker(0, () -> {

                //timer.reset();
                //linearSlide.setPower(-1.0);
                //while  (timer.milliseconds() < 2500.0) {

                //}
                //linearSlide.setPower(0);


              })
              .build();

      Trajectory autoTrajectory8 = drive.trajectoryBuilder(autoTrajectory7.end().plus(new Pose2d(0,0,Math.toRadians(180))))
              .forward(24.0)
              .build();

      Trajectory autoTrajectory9 = drive.trajectoryBuilder(autoTrajectory8.end())
              .lineToLinearHeading(new Pose2d(-36, 28, Math.toRadians(360)))
              .build();

      Trajectory autoTrajectory10 = drive.trajectoryBuilder(autoTrajectory9.end())
              .strafeLeft(16.0)
              .build();

      drive.followTrajectory(
              // drops cone at ground junction
              autoTrajectory1);
      Thread.sleep(1500);
      drive.followTrajectory(
              // moves backward 6 inches
              autoTrajectory2);
      drive.followTrajectory(
              // strafe right 3 inches
              autoTrajectory5);
      drive.turn(Math.toRadians(90));
      drive.followTrajectory(
              // grabs detection cone
              autoTrajectory3);
      Thread.sleep(500);
      //linearSlide.setPower(1.0);
      Thread.sleep(5250);
      //linearSlide.setPower(0);
      //drive.turn(Math.toRadians(45));
      drive.followTrajectory(
              // goes to mid junction and drops cone
              autoTrajectory4);
      Thread.sleep(1500);
      drive.turn(Math.toRadians(180));
      Thread.sleep(250);
      drive.followTrajectory(
              // strafes left
              autoTrajectory6);
      drive.followTrajectory(
              // goes to stack of cones
              autoTrajectory7);
      drive.turn(Math.toRadians(180));
      drive.followTrajectory(
              // moves forward 12 inches
              autoTrajectory8);
      drive.followTrajectory(
              // goes to mid junction round 2
              autoTrajectory9);
      drive.followTrajectory(
              // goes to zone 2
              autoTrajectory10);

      /* Actually do something useful */
      if (tagOfInterest == null || tagOfInterest.id == LEFT) {

        //trajectory
        Trajectory zone1 = drive.trajectoryBuilder(new Pose2d(-36,36,Math.toRadians(360)))
                .forward(16.0)
                .build();

        drive.followTrajectory(zone1);

      } else if (tagOfInterest.id == RIGHT) {

        //trajectory
        Trajectory zone2 = drive.trajectoryBuilder(new Pose2d(-36,36,Math.toRadians(360)))
                .back(16.0)
                .build();

        drive.followTrajectory(zone2);

      } else {

        // right trajectory
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

  /*
  public void runOpMode() throws InterruptedException {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    CRServo grabberServo;

    ElapsedTime timer = new ElapsedTime();

    grabberServo = hardwareMap.get(CRServo.class, "Grabber");

    //Starting the robot at the bottom left (blue auto)
    Pose2d startPose = new Pose2d(-36,72,Math.toRadians(270));
    drive.setPoseEstimate(startPose);

    double slowerVelocity = 24.0;

   */
      //linearSlide.setPower(-1.0);
    //Thread.sleep(5500);
    //linearSlide.setPower(0);





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



  }



// .strafeTo(new Vector2d(-54.0, 60.0))
//Step 2: Pick up detection cone and drop it off at medium junction
      /*  .splineTo(new Vector2d(-36, 54), 90)
        .splineTo(new Vector2d(-36, 24), 0)
        //Step 3: Pick up and drop off cones at the low junction by the cone station (x3)
        .splineTo(new Vector2d(-60, 12), 180)
        .splineTo(new Vector2d(-48, 12), -90)
        .splineTo(new Vector2d(-60, 12), 180)
        .splineTo(new Vector2d(-48, 12), -90)
        .splineTo(new Vector2d(-60, 12), 180)
        .splineTo(new Vector2d(-48, 12), -90)
        //Step 4 pick up cone and drop it off it high junction at zone 2 and 1
        .splineTo(new Vector2d(-60, 1), 180)
        .splineTo(new Vector2d(-24, 1), 90) */

    /*
    //Runs the path you made
    drive.followTrajectory(autoTrajectory1);
    Thread.sleep(1000);
    drive.followTrajectory(autoTrajectory2);
    drive.followTrajectory(autoTrajectory3);


 Trajectory autoTrajectory = drive.trajectoryBuilder(startPose)

            //step 1 go to ground junction
            .splineTo(new Vector2d(-40.0, 54.0), Math.toRadians(180.0))
            .addDisplacementMarker(() -> {
              // Run your action in here!
              //grabberServo.setPower(1.0);
            })

            //step 2 go pick up signal sleeve cone
            .splineTo(new Vector2d(-34.0, 54.0), Math.toRadians(180.0))
            .splineTo(new Vector2d(-34.0, 48.0), Math.toRadians(270.0))
            .addDisplacementMarker(() -> {
              // Run your action in here!
              //grabberServo.setPower(-1.0);
            })

            //step 3 go to mid junction
            .splineTo(new Vector2d(-24.0, 24.0), Math.toRadians(315.0))
            .addDisplacementMarker(() -> {
              // Run your action in here!
              //linearSlide.setPower(1.0);
              //sleep
              //linearSlide.setPower(0.0);

              //grabberServo.setPower(1.0);
            })
            .build();
    */