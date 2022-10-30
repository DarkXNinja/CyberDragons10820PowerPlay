import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunnerConfiguration.drive.SampleMecanumDrive;

public class RoadRunnnerTest extends LinearOpMode {

  public void runOpMode() {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //Makes the path for the robot to follow
    //.splineTo(new Vector2d(xCord, yCord), rotation)
    Trajectory autoTrajectory = drive.trajectoryBuilder(new Pose2d(-3, 6, Math.toRadians(90)))
        //Step 1: Drop off cone at low juction
        .splineTo(new Vector2d(-3, 4), 0)
        //Step 2: Pick up detection cone and drop it off at medium junction
        .splineTo(new Vector2d(-3, 4.5), 90)
        .splineTo(new Vector2d(-3, 2), 0)
        //Step 3: Pick up and drop off cones at the low junction by the cone station (x3)
        .splineTo(new Vector2d(-5, 1), 180)
        .splineTo(new Vector2d(-4, 1), -90)
        .splineTo(new Vector2d(-5, 1), 180)
        .splineTo(new Vector2d(-4, 1), -90)
        .splineTo(new Vector2d(-5, 1), 180)
        .splineTo(new Vector2d(-4, 1), -90)
        //Step 4 pick up cone and drop it off it high juntion at zone 2 and 1
        .splineTo(new Vector2d(-5, 1), 180)
        .splineTo(new Vector2d(-2, 1), 90)
        .build();
        
    waitForStart();

    if(isStopRequested()) return;
    
    //Runs the path you made
    drive.followTrajectory(autoTrajectory);
    
;
  }
}
