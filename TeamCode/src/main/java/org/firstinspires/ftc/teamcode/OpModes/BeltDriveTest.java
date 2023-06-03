package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class BeltDriveTest extends LinearOpMode {

    //drivetrain motors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;

    // lift motors
    private DcMotorEx lift1;
    private DcMotorEx lift2;
    private Servo grabberservo;

    // sensors
    private DistanceSensor gripperHeight; // sensor for detecting height of gripper
    private DistanceSensor junctionSensor; // sensor for detecting distance to junction
    private ColorSensor coneSensor ;
    private DistanceSensor dconeSensor ; // repurposing the color sensor as distance sensor

    double speedVal = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Starting to intialize..");
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
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // really important if using normal mode

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // really important if using normal mode

        // initializing the servo
        grabberservo = hardwareMap.get(Servo.class, "GrabberServo");
        closeGripper();

        // initializing the sensors
        gripperHeight = hardwareMap.get(DistanceSensor.class, "gripperHeight");
        junctionSensor = hardwareMap.get(DistanceSensor.class, "junctionSensor");
        coneSensor = hardwareMap.get(ColorSensor.class, "coneSensor");
        dconeSensor = hardwareMap.get(DistanceSensor.class, "coneSensor");

        telemetry.addLine("Initialized. Press START to begin TeleOp..");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("lift1 encoder: " + lift1.getCurrentPosition());
            telemetry.addLine("lift2 encoder: " + lift2.getCurrentPosition());
            telemetry.addLine("gripper height: " + gripperHeight.getDistance(DistanceUnit.INCH));
            telemetry.addLine("junction distance: " + junctionSensor.getDistance(DistanceUnit.INCH));
            telemetry.addLine("cone colors: R: " + coneSensor.red() + " G: " + coneSensor.green() + " B: " + coneSensor.blue());
            telemetry.addLine("cone distance: " + dconeSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();

            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                if (gamepad1.dpad_up) {
                    moveForward(0.2) ;
                }
                if (gamepad1.dpad_down) {
                    moveBackward(0.2);
                }
                if (gamepad1.dpad_left) {
                    strafeLeft(0.2);
                }
                if (gamepad1.dpad_right) {
                    strafeRight(0.2) ;
                }
            } else {
                // for the drive train
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

                frontLeft.setPower(frontLeftPower*speedVal);
                backLeft.setPower(backLeftPower*speedVal);
                frontRight.setPower(frontRightPower*speedVal);
                backRight.setPower(backRightPower*speedVal);
                /*
                frontLeft.setPower(Math.pow(frontLeftPower,3)*speedVal);
                backLeft.setPower(Math.pow(backLeftPower,3)*speedVal);
                frontRight.setPower(Math.pow(frontRightPower,3)*speedVal);
                backRight.setPower(Math.pow(backRightPower,3)*speedVal);
                 */
            }

            if ((gripperHeight.getDistance(DistanceUnit.INCH) < 3.0) && (gamepad2.left_stick_y > 0)) {
                // stop for the low limit
                lift1.setPower(0.0);
                lift2.setPower(0.0);
            } else {
               if ((lift1.getCurrentPosition() < -5500) && (gamepad2.left_stick_y < 0)) {
                   // stop for the high limit
                    lift1.setPower(0.0);
                    lift2.setPower(0.0);
                }
                else {
                    // in all other cases;
                    lift1.setPower(gamepad2.left_stick_y);
                    lift2.setPower(gamepad2.left_stick_y);
                }
            }

            // for the lift
            // Then then grabber
            if(gamepad2.x) {
                closeGripper();
            }
            if(gamepad2.y) {
                openGripper();
            }

            if (gamepad2.dpad_down) {
                if (gripperHeight.getDistance(DistanceUnit.INCH) > 3.0) {
                    lift1.setPower(0.8);
                    lift2.setPower(0.8);
                    while (gripperHeight.getDistance(DistanceUnit.INCH) > 3.2) ;
                    lift1.setPower(0.0);
                    lift2.setPower(0.0);
                }
                lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // really important if using normal mode

                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // really important if using normal mode
            }

        }
    }

    private void openGripper() {
        grabberservo.setPosition(0); // open gripper
    }
    private void closeGripper() {
        grabberservo.setPosition(1); // close gripper
    }

    void moveForward(double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);
    }

    void moveBackward(double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }

    void strafeLeft(double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
    }

    void strafeRight(double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
    }

    void stopAllWheels() {

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    void upLift() {

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    void downLift() {

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

}

