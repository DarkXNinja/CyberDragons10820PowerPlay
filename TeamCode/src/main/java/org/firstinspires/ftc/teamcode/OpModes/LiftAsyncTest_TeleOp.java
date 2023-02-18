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
public class LiftAsyncTest_TeleOp extends LinearOpMode {

    //drivetrain motors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;

    private DcMotorEx lift1;
    private DcMotorEx lift2;

    private DistanceSensor gripperHeight;

    final int ttolerance = 25;

    double speedVal = 0.6;

    enum LiftMode {
        DRIVER_CONTROL, //player controlling it
        AUTOMATIC_CONTROL //Robot controlling it
    }

    LiftMode currentLiftMode = LiftMode.DRIVER_CONTROL ;

    ElapsedTime durationTimer = new ElapsedTime();

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

        gripperHeight = hardwareMap.get(DistanceSensor.class, "gripperHeight");

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

            // for the rest
            if (currentLiftMode == LiftMode.DRIVER_CONTROL) {
                if (gamepad2.left_stick_y > 0) {
                    // Positive Y val means it is being pressed down
                    telemetry.addLine("Gamepad2 Left Stick: " + gamepad2.left_stick_y);
                    telemetry.update();

                    // if the gripper height is lower than a value, then dont allow the lift to be driven
                    if (gripperHeight.getDistance(DistanceUnit.INCH) > 1.5) {
                        lift1.setPower(gamepad2.left_stick_y * 0.75);
                        lift2.setPower(gamepad2.left_stick_y * 0.75);

                    } else {
                        lift1.setPower(0.0);
                        lift2.setPower(0.0);
                    }
                } else {
                    telemetry.addLine("Gamepad2 Left Stick: " + gamepad2.left_stick_y);
                    telemetry.update();

                    lift1.setPower(gamepad2.left_stick_y * 0.75);
                    lift2.setPower(gamepad2.left_stick_y * 0.75);

                }
            }

            if (gamepad1.right_bumper){

                resetLiftEncoder();

            }


            if (gamepad2.dpad_up) {

                moveLiftToPositionAsync(-3300);

            }

            if (gamepad1.left_bumper) {
                lift1.setPower(0.0) ;
                lift2.setPower(0.0);
                lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                currentLiftMode = LiftMode.DRIVER_CONTROL ;
            }

        }
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

        // by default, we will make it driver control upon reset
        currentLiftMode = LiftMode.DRIVER_CONTROL ;
    }

    void moveLiftToPosition(int pos) {
        int tposition = pos;
        final int ttolerance = 25;
        int thigher = tposition - ttolerance;
        int tlower = tposition + ttolerance;

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // really important if using normal mode

        // When not using async; revert back to driver control
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

        // make sure you call checkListInPosition to make sure lift is in position
        currentLiftMode = LiftMode.AUTOMATIC_CONTROL ;
        // it is removed from automatic control, when gamepad1.left_bumper is pressed
        // or when robot is reset using right bumper
    }


}
