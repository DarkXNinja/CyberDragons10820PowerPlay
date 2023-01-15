package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class PowerPlayTeleOp extends LinearOpMode {

    //drivetrain motors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;

    private DcMotorEx lift1;
    private DcMotorEx lift2;
    private Servo grabberservo;
    private CRServo gripperrotator;

    private DistanceSensor gripperHeight;
    private TouchSensor magTouch;

    ElapsedTime durationTimer = new ElapsedTime();

    private String gripperPosition;

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

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        //lift1.setTargetPosition(0);
        //lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        //lift2.setTargetPosition(0);
        //lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabberservo = hardwareMap.get(Servo.class, "GrabberServo");
        grabberservo.setPosition(1);
        gripperrotator = hardwareMap.get(CRServo.class, "GripperRotator");

        gripperHeight = hardwareMap.get(DistanceSensor.class, "gripperHeight");
        magTouch = hardwareMap.get(TouchSensor.class, "touch");

        if (magTouch.isPressed()) {
            gripperPosition = "center";

        }

        waitForStart();

        durationTimer.reset();

        while (opModeIsActive()) {

            // for the drive train
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * -1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower * 0.5);
            backLeft.setPower(backLeftPower * 0.5);
            frontRight.setPower(frontRightPower * 0.5);
            backRight.setPower(backRightPower * 0.5);


            // for the rest
            if (gripperHeight.getDistance(DistanceUnit.INCH) > 1) {

                lift1.setPower(gamepad2.left_stick_y);
                lift2.setPower(gamepad2.left_stick_y);

            } else {

                lift1.setPower(0);
                lift2.setPower(0);

            }

            if(gamepad2.x) {
                grabberservo.setPosition(1);
            }
            if(gamepad2.y) {
                grabberservo.setPosition(0);
            }

            if (gamepad2.dpad_right) {

                gripperrotator.setPower(1.0*0.5);
                gripperPosition = "right";

            } else if (gamepad2.dpad_left) {

                gripperrotator.setPower(-1.0 * 0.5);
                gripperPosition = "left";

            } else {

                gripperrotator.setPower(0.0);

            }

            // centering automation for gripper
            if (gamepad2.dpad_down) {

                if (gripperPosition == "center") {

                    gripperrotator.setPower(0.0);

                } else if (gripperPosition == "right") {

                    gripperrotator.setPower(-1.0 * 0.5);
                    while (!magTouch.isPressed()) {

                    }
                    gripperrotator.setPower(0.0);

                } else if (gripperPosition == "left") {

                    gripperrotator.setPower(1.0*0.5);
                    while (!magTouch.isPressed()) {

                    }
                    gripperrotator.setPower(0.0);

                }

            }



            if (durationTimer.milliseconds() > 1000) {
                telemetry.addLine("Height: " + gripperHeight.getDistance(DistanceUnit.INCH));
                if (magTouch.isPressed()) {
                    telemetry.addLine("Touch: " + "YES");
                } else {
                    telemetry.addLine("Touch: " + "NO");
                }
                telemetry.addLine("lift1 encoder: " + lift1.getCurrentPosition());
                telemetry.addLine("lift2 encoder: " + lift2.getCurrentPosition());
                telemetry.update();
                // reset timer
                durationTimer.reset();
            }
            //Needs values
            /*
            A = low
            B = High
            The lower one on the controller is lower and the higher is high
            */
            if (gamepad2.a) {
              moveLiftToPosition(-3300);
            }
            if (gamepad2.b){
              moveLiftToPosition(-17500);
            }
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
    }
}
