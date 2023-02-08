package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    ElapsedTime durationTimer = new ElapsedTime();

    ElapsedTime centeringTimer = new ElapsedTime();

    final int ttolerance = 25;

    double speedVal = 0.6;


    //private String gripperPosition;

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

        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // setting up the rest
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

        //reversing lift 2
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        //lift1.setTargetPosition(0);
        //lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        //lift2.setTargetPosition(0);
        //lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabberservo = hardwareMap.get(Servo.class, "GrabberServo");


        gripperfolder = hardwareMap.get(Servo.class, "GripperFolder");

        gripperHeight = hardwareMap.get(DistanceSensor.class, "gripperHeight");
        rightPole = hardwareMap.get(DistanceSensor.class, "rightPole");

        grabberservo.setPosition(1);
        gripperfolder.setPosition(0);

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

            // for the rest
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

            if(gamepad2.x) {
                grabberservo.setPosition(1);
            }
            if(gamepad2.y) {
                grabberservo.setPosition(0);
            }

            if (gamepad2.left_bumper) {

                gripperfolder.setPosition(1.0);

            } else if (gamepad2.right_bumper) {

                gripperfolder.setPosition(0.0);

            }


            if (durationTimer.milliseconds() > 1000) {
                telemetry.addLine("Height: " + gripperHeight.getDistance(DistanceUnit.INCH));
                telemetry.addLine("pole distance right: " + rightPole.getDistance(DistanceUnit.INCH));

                telemetry.addLine("lift1 encoder: " + lift1.getCurrentPosition());
                telemetry.addLine("lift2 encoder: " + lift2.getCurrentPosition());
                telemetry.update();
                // reset timer
                durationTimer.reset();
            }


            if (gamepad1.right_bumper){

                resetLiftEncoder();

            }


            if (gamepad2.dpad_down) {

                moveLiftToPosition(-1000);

            }

            if (gamepad2.dpad_up) {

                moveLiftToPosition(-3300);

            }

            if (gamepad2.dpad_right) {

                moveLiftToPosition(-2000);

            }

            if (gamepad1.dpad_left) {

                //adjustRobotPositionToJunction();
            }

            if (gamepad1.left_bumper) {
                lift1.setPower(0.0) ;
                lift2.setPower(0.0);
                lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                //adjustRobotPositionToJunction();
            }


            //Needs values
            /*
            A = low
            B = High
            High = -6600
            Medium = -3000
            Low = -1800
            The lower one on the controller is lower and the higher is high
            */

            /*
            if (gamepad2.a) {
              moveLiftToPosition(-1800);
            }


            if (gamepad2.b){
              moveLiftToPosition(-6600);
            }
            */
			/*
			if (gamepad2.a) {

				int tposition = -3150 ;
				int ttolerance = 50 ;

				int thigher = tposition - ttolerance ;
				int tlower = tposition + ttolerance ;

				lift1.setTargetPosition(tposition);
				lift1.setTargetPositionTolerance(ttolerance);
				lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				lift2.setTargetPosition(tposition);
				lift2.setTargetPositionTolerance(ttolerance);
				lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				lift1.setPower(0.5) ;
				lift2.setPower(0.5) ;

				while (true) {
					int lift1position = lift1.getCurrentPosition() ;
					int lift2position = lift2.getCurrentPosition() ;

					if ( ((lift1position < tlower) && (lift1position > thigher)) ||
								 ((lift2position < tlower) && (lift2position > thigher)) ) {

					   lift1.setPower(0.0) ;
					   lift2.setPower(0.0) ;
					   break ;
					}
				}

			}

			if (gamepad2.b) {

				int tposition = -5150 ;
				int ttolerance = 50 ;

				int thigher = tposition - ttolerance ;
				int tlower = tposition + ttolerance ;

				lift1.setTargetPosition(tposition);
				lift1.setTargetPositionTolerance(ttolerance);
				lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				lift2.setTargetPosition(tposition);
				lift2.setTargetPositionTolerance(ttolerance);
				lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				lift1.setPower(0.5) ;
				lift2.setPower(0.5) ;

				while (true) {
					int lift1position = lift1.getCurrentPosition() ;
					int lift2position = lift2.getCurrentPosition() ;

					if ( ((lift1position < tlower) && (lift1position > thigher)) ||
								 ((lift2position < tlower) && (lift2position > thigher)) ) {

					   lift1.setPower(0.0) ;
					   lift2.setPower(0.0) ;
					   break ;
					}
				}

			}

			if (gamepad1.a) {

				int tposition = -7150 ;
				int ttolerance = 50 ;

				int thigher = tposition - ttolerance ;
				int tlower = tposition + ttolerance ;

				lift1.setTargetPosition(tposition);
				lift1.setTargetPositionTolerance(ttolerance);
				lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				lift2.setTargetPosition(tposition);
				lift2.setTargetPositionTolerance(ttolerance);
				lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				lift1.setPower(0.5) ;
				lift2.setPower(0.5) ;

				while (true) {
					int lift1position = lift1.getCurrentPosition() ;
					int lift2position = lift2.getCurrentPosition() ;

					if ( ((lift1position < tlower) && (lift1position > thigher)) ||
								 ((lift2position < tlower) && (lift2position > thigher)) ) {

					   lift1.setPower(0.0) ;
					   lift2.setPower(0.0) ;
					   break ;
					}
				}

			}

			if (gamepad1.b) {

				int tposition = -50 ;
				int ttolerance = 50 ;

				int thigher = tposition - ttolerance ;
				int tlower = tposition + ttolerance ;

				lift1.setTargetPosition(tposition);
				lift1.setTargetPositionTolerance(ttolerance);
				lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				lift2.setTargetPosition(tposition);
				lift2.setTargetPositionTolerance(ttolerance);
				lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				lift1.setPower(0.5) ;
				lift2.setPower(0.5) ;

				while (true) {
					int lift1position = lift1.getCurrentPosition() ;
					int lift2position = lift2.getCurrentPosition() ;

					if ( ((lift1position < tlower) && (lift1position > thigher)) ||
								 ((lift2position < tlower) && (lift2position > thigher)) ) {

					   lift1.setPower(0.0) ;
					   lift2.setPower(0.0) ;
					   break ;
					}
				}

			}
			*/
        }


    }

    void moveLiftToPosition(int pos) {
      int tposition = pos;
      final int ttolerance = 25;
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

        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode

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

        /*
        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

         */
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


    }

    void moveLiftToPositionAsync(int pos) {
        int tposition = pos;
        int thigher = tposition - ttolerance;
        int tlower = tposition + ttolerance;

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setTargetPosition(tposition);
        lift1.setTargetPositionTolerance(ttolerance);
        lift2.setTargetPosition(tposition);
        lift2.setTargetPositionTolerance(ttolerance);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setVelocity(150);
        lift2.setVelocity(150);

        // make sure you call checkListInPosition to make sure lift is in position
    }




}
