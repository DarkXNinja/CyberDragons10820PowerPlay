public class MyOpmode extends LinearOpMode {
  public void runOpMode() {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //Makes the path for the robot to follow
    //.splineTo(new Vector2d(xCord, yCord), rotation)
    Trajectory autoTrajectory = drive.trajectoryBuilder(new Pose2d(-6, -3, Math.toRadians(90)))
        //Step 1: Drop off cone at low juction
        .splineTo(new Vector2d(-4, -3), 0)
        //Step 2: Pick up detection cone and drop it off at medium junction
        .splineTo(new Vector2d(-4.5, -3), 90)
        .splineTo(new Vector2d(-2, -3, 0)
        //Step 3: Pick up and drop off cones at the low junction by the cone station (x3)
        .splineTo(new Vector2d(-1, -5), 180)
        .splineTo(new Vector2d(-1, -4), -90)
        .splineTo(new Vector2d(-1, -5), 180)
        .splineTo(new Vector2d(-1, -4), -90)
        .splineTo(new Vector2d(-1, -5), 180)
        .splineTo(new Vector2d(-1, -4), -90)
        //Step 4 pick up cone and drop it off it high juntion at zone 2 and 1
        .splineTo(new Vector2d(-1, -5), 180)
        .splineTo(new Vector2d(-1, -2), 90)
        .build();
        
    waitForStart();

    if(isStopRequested()) return;
    
    //Runs the path you made
    drive.followTrajectory(autoTrajectory);
    
;
  }
}
