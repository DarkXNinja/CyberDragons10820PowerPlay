package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.RoadRunnerConfiguration.drive.SampleMecanumDrive;

@Autonomous
public class RoadRunnerTest extends LinearOpMode {

  public void runOpMode() throws InterruptedException {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    CRServo grabberServo;
    grabberServo = hardwareMap.get(CRServo.class, "grabber");

    //Starting the robot at the bottom left (blue auto)
    Pose2d startPose = new Pose2d(-36,72,Math.toRadians(270));
    drive.setPoseEstimate(startPose);


    waitForStart();
    if (isStopRequested()) return;


    //Makes the path for the robot to follow
    //.splineTo(new Vector2d(xCord, yCord), rotation)
    Trajectory autoTrajectory1 = drive.trajectoryBuilder(startPose)
        //Step 1: Drop off cone at low junction
            //.forward(24.0)
            .lineToLinearHeading(new Pose2d(-40, 54, Math.toRadians(180)))
            .addDisplacementMarker(() -> {
              // Run your action in here!
              grabberServo.setPower(1.0);
            })
            .build();

    Trajectory autoTrajectory2 = drive.trajectoryBuilder(autoTrajectory1.end())
            .lineToLinearHeading(new Pose2d(-36, 72, Math.toRadians(270)))
            //.back(6.0)
            .build();

    Trajectory autoTrajectory3 = drive.trajectoryBuilder(autoTrajectory2.end())
            //.lineToLinearHeading(new Pose2d(-34,58, Math.toRadians(270)))
            .forward(24.0)
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
        .build();

    
    //Runs the path you made
    drive.followTrajectory(autoTrajectory1);
    Thread.sleep(1000);
    drive.followTrajectory(autoTrajectory2);
    drive.followTrajectory(autoTrajectory3);

    ;
  }
}
