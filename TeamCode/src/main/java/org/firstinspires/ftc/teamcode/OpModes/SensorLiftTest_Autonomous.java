package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class SensorLiftTest_Autonomous extends LinearOpMode {

    // lift motors
    DcMotorEx lift1;
    DcMotorEx lift2;

    // sensors
    private DistanceSensor gripperHeight; // sensor for detecting height of gripper
    private DistanceSensor junctionSensor; // sensor for detecting distance to junction


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Starting to intialize..");
        telemetry.update();

        initializeRobot();

        waitForStart();

        if (opModeIsActive()) {

        }
    }

    void initializeRobot() {
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
    }

}
