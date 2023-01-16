package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class InitializationClass {

    //drivetrain motors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;


    public void initializeDrivetrain(HardwareMap hardware) {

        frontRight = hardware.get(DcMotorEx.class, "frontRight");
        frontLeft = hardware.get(DcMotorEx.class, "frontLeft");
        backRight = hardware.get(DcMotorEx.class, "backRight");
        backLeft = hardware.get(DcMotorEx.class, "backLeft");

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void initializeIMU(HardwareMap hardware) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode	= BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // get and initialize the IMU
        // The imu is assumed to be on I2C port
        // and configured to be a sensor of type "AdaFruit IMU" and named "imu"
        //imu = hardware.get(BNO055IMU.class, "imu");

        //imu.initialize(parameters);

        /*
        telemetry.addData("Mode", "IMU calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", "Calibrated" + imu.getCalibrationStatus().toString());
        telemetry.update( );
         */

    }

    public void moveForward(int duration, double power) throws InterruptedException {

        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);

        Thread.sleep(duration);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);


    }

}
