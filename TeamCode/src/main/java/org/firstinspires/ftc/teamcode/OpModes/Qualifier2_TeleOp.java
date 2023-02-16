package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@TeleOp
public class Qualifier2_TeleOp extends LinearOpMode {

    //drivetrain motors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;

    private DcMotorEx lift1;
    private DcMotorEx lift2;
    private Servo grabberservo;
    private Servo gripperfolder;

    private DistanceSensor gripperHeight;
    private DistanceSensor rightPole;
    private DistanceSensor junctionSensor;

    ElapsedTime durationTimer = new ElapsedTime();

    ElapsedTime centeringTimer = new ElapsedTime();

    final int ttolerance = 25;

    double speedVal = 0.6;

    enum LiftMode {
        DRIVER_CONTROL, //player controlling it
        AUTOMATIC_CONTROL //Robot controlling it
    }
    LiftMode currentLiftMode = LiftMode.DRIVER_CONTROL ;

    final int LIFT_LOW = -1000 ;
    final int LIFT_MEDIUM = -2000 ;
    final int LIFT_HIGH = -3300 ;
    final int LIFT_GROUND = -200 ;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("reached");
        telemetry.update();

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

        // setting up the lift
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

        //reversing lift 2
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        currentLiftMode = LiftMode.DRIVER_CONTROL ;

        // set up the servos and position them
        grabberservo = hardwareMap.get(Servo.class, "GrabberServo");
        gripperfolder = hardwareMap.get(Servo.class, "GripperFolder");
        closeGripper();
        upGripper(); ;

        // set up the sensors
        gripperHeight = hardwareMap.get(DistanceSensor.class, "gripperHeight");
        rightPole = hardwareMap.get(DistanceSensor.class, "rightPole");
        junctionSensor = hardwareMap.get(DistanceSensor.class, "junctionSensor");

        waitForStart();

        durationTimer.reset();

        while (opModeIsActive()) {

            // for the drive train
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * -1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x * 0.75;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of  the range [-1, 1]


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower * speedVal);
            backLeft.setPower(backLeftPower * speedVal);
            frontRight.setPower(frontRightPower * speedVal);
            backRight.setPower(backRightPower * speedVal);

            if (gamepad1.x) {

                if (speedVal == 0.6) {

                    speedVal = 1.0;

                } else {

                    speedVal = 0.6;
                }

            }

            // check if lift is in DRIVER or AUTOMATED mode
            if (currentLiftMode == LiftMode.DRIVER_CONTROL) {
                // NOTE:Y-axis of the joysticks is negative when pushed up, and positive when pushed down
                if (gamepad2.left_stick_y > 0) {
                    if (gripperHeight.getDistance(DistanceUnit.INCH) > 1.5) {

                        lift1.setPower(gamepad2.left_stick_y * 0.75);
                        lift2.setPower(gamepad2.left_stick_y * 0.75);

                    } else {
                        lift1.setPower(0.0);
                        lift2.setPower(0.0);
                    }
                } else {
                    lift1.setPower(gamepad2.left_stick_y * 0.75);
                    lift2.setPower(gamepad2.left_stick_y * 0.75);
                }
            }

            // bumper can be used to get reset lift or get it out of AUTO mode
            if (gamepad1.right_bumper){
                resetLiftEncoder();
            }
            if (gamepad1.left_bumper) {
                exitLiftAutomatic();
            }

            // then for the different junction heights
            // Note sync functions being used
            // exitAutomatic or resetGripper to get to driver control
            if (gamepad2.dpad_down) {
                moveLiftToPositionAsync(LIFT_LOW); // low junction
            }

            if (gamepad2.dpad_right) {
                moveLiftToPositionAsync(LIFT_MEDIUM); // medium junction
            }

            if (gamepad2.dpad_up) {
                moveLiftToPositionAsync(LIFT_HIGH); // high junction
            }

            if (gamepad2.dpad_left) {
                moveLiftToPositionAsync(LIFT_GROUND); // ground junction
            }

            // Then then grabber
            if(gamepad2.x) {
                closeGripper();
            }
            if(gamepad2.y) {
                openGripper();
            }

            if (gamepad2.left_bumper) {
                downGripper();
            }

            if (gamepad2.right_bumper) {
                upGripper();
            }

/*
            if (durationTimer.milliseconds() > 1000) {
                telemetry.addLine("Height: " + gripperHeight.getDistance(DistanceUnit.INCH));
                telemetry.addLine("pole distance right: " + rightPole.getDistance(DistanceUnit.INCH));
                telemetry.addLine("junction distance" + junctionSensor.getDistance(DistanceUnit.INCH));

                telemetry.addLine("lift1 encoder: " + lift1.getCurrentPosition());
                telemetry.addLine("lift2 encoder: " + lift2.getCurrentPosition());

                telemetry.update();
                // reset timer
                durationTimer.reset();
            }

 */

        }

    }

    void moveLiftToPosition(int pos) {
      int tposition = pos;
      final int ttolerance = 25;
      int thigher = tposition - ttolerance;
      int tlower = tposition + ttolerance;

      lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      lift1.setTargetPosition(tposition);
      lift1.setTargetPositionTolerance(ttolerance);
      lift2.setTargetPosition(tposition);
      lift2.setTargetPositionTolerance(ttolerance);

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

      // this is not async ; so keep it driver controlled
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        currentLiftMode = LiftMode.DRIVER_CONTROL ;

    }

    public void resetLiftEncoder() {

        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode

        lift1.setPower(0.5);
        lift2.setPower(0.5);
        while (gripperHeight.getDistance(DistanceUnit.INCH) > 1.5) {

        }
        lift1.setPower(0.0);
        lift2.setPower(0.0);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode

        // make it driver controlled
        currentLiftMode = LiftMode.DRIVER_CONTROL ;

    }

    void moveLiftToPositionAsync(int pos) {
        int tposition = pos;
        int thigher = tposition - ttolerance;
        int tlower = tposition + ttolerance;

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setTargetPosition(tposition);
        lift1.setTargetPositionTolerance(ttolerance);
        lift2.setTargetPosition(tposition);
        lift2.setTargetPositionTolerance(ttolerance);

        lift1.setPower(0.75);
        lift2.setPower(0.75);

        // the only time when it is Automatic control
        currentLiftMode = LiftMode.AUTOMATIC_CONTROL ;
    }

    void exitLiftAutomatic() {
        lift1.setPower(0.0) ;
        lift2.setPower(0.0);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentLiftMode = LiftMode.DRIVER_CONTROL ;
    }

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

        /*
        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


         */

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

    /*
    public boolean adjustRobotPositionToJunction() throws InterruptedException {

        grabberservo.setPosition(1);


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


    private void openGripper() {
        grabberservo.setPosition(0); // open gripper
    }
    private void closeGripper() {
        grabberservo.setPosition(1); // close gripper
    }
    private void upGripper() {
        gripperfolder.setPosition(0); // up gripper
    }
    private void downGripper() {
        gripperfolder.setPosition(1); // down gripper
    }

}
