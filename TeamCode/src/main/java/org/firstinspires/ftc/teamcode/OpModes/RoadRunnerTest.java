package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunnerConfiguration.drive.SampleMecanumDrive;

@Autonomous
public class RoadRunnerTest extends LinearOpMode {

  public void runOpMode() throws InterruptedException {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //Starting the robot at the bottom left (blue auto)
    Pose2d startPose = new Pose2d(-36,72,Math.toRadians(90));
    drive.setPoseEstimate(startPose);


    waitForStart();
    if (isStopRequested()) return;


    //Makes the path for the robot to follow
    //.splineTo(new Vector2d(xCord, yCord), rotation)
    Trajectory autoTrajectory = drive.trajectoryBuilder(startPose)
        //Step 1: Drop off cone at low junction
        .splineTo(new Vector2d(-36, 48), 0)
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
    drive.followTrajectory(autoTrajectory);
    
;
  }
}
