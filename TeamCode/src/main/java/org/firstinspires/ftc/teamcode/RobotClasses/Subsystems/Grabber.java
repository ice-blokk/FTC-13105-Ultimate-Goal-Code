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

        grabberOpen = 90;
        grabberClosed = -10;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void armDown() {
        if(arm.getCurrentPosition() < 180) {
            arm.setPower(.5);
        }
    }

    public void armUp() {
        if(arm.getCurrentPosition() > 0) {
            arm.setPower(-.5);
        }
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
