package org.firstinspires.ftc.teamcode.RobotClasses.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    private DcMotor arm;
    private Servo grabber;
    private final double grabberOpen, grabberClosed;

    public Grabber(DcMotor arm, Servo grabber) {
        this.arm = arm;
        this.grabber = grabber;

        grabberOpen = 0;
        grabberClosed = 90;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void down() {
            arm.setPower(.25);
    }

    public void up() {
            arm.setPower(-.25);
    }

    public void armSetDown() {
        arm.setTargetPosition(1500);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void armSetUp() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void off() {
        arm.setPower(0);
    }

    public void resetArmEncoder() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void openGrabber() {
        grabber.setPosition(grabberOpen);
    }

    public void closeGrabber() {
        grabber.setPosition(grabberClosed);
    }

}
