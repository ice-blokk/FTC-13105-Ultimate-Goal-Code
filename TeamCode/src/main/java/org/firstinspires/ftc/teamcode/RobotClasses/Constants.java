package org.firstinspires.ftc.teamcode.RobotClasses;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class Constants extends OpMode {

    // Drive motors
    public final DcMotor
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeftDrive"),
            frontRight = hardwareMap.get(DcMotor.class, "frontRightDrive"),
            backLeft = hardwareMap.get(DcMotor.class, "backLeftDrive"),
            backRight = hardwareMap.get(DcMotor.class, "backRightDrive");

    public final DcMotor
            leftOdometer = hardwareMap.get(DcMotor.class, "frontLeftDrive"),
            rightOdometer = hardwareMap.get(DcMotor.class, "frontRightDrive"),
            backOdometer = hardwareMap.get(DcMotor.class, "backLeftDrive");

    public final DcMotor
            intake = hardwareMap.get(DcMotor.class, "intakeMotor"),
            arm = hardwareMap.get(DcMotor.class, "armMotor");

    public final DcMotorEx
            shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");

    public final Servo
            indexer = hardwareMap.get(Servo.class, "indexerServo"),
            grabber = hardwareMap.get(Servo.class, "grabberServo");

    public final BNO055IMU
            imu = hardwareMap.get(BNO055IMU.class, "imu");


    @Override
    public void init() {}

    @Override
    public void loop() {}
}
