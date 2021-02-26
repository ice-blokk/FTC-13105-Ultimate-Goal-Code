package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Class for simple control of a mecanum drivetrain.
 * NOTE: The HD Hex Motors with the UltraPlanetary GearBox Kit has a free speed of 6000 RPM.
 * The gear ratio on our driveetrain is 15:1 so the max theoretical free speed is 400 RPM.
 */
public class MecanumDrive {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private Gyro gyro;

    private double maxSpeedCap;
    private double speedMultiplier;

    public MecanumDrive(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, Gyro gyro) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;

        this.gyro = gyro;

        maxSpeedCap = 1;
        speedMultiplier = 1;
    }

    /**
     * Control the mecanum drivetrain using forward, strafe, and turn parameters
     * @param forward move forward (positive value) or backward (negative value)
     * @param strafe move right (positive) or left (negative)
     * @param turn turn clock-wise (positive) or counter-clockwise (negative)
     */
    public void drive(double forward, double strafe, double turn, boolean isFieldOriented) {
        double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

        double y = -forward; // this is reversed
        double x = strafe;
        double rx = turn;


        if(isFieldOriented) {
            double angle = Math.toRadians(gyro.getAngle());
            double temp = -y * Math.cos(angle) + x * Math.sin(angle);
            x = y * Math.sin(angle) + x * Math.cos(angle);
            y = temp;
        }

        frontLeftPower = y + x - rx;
        backLeftPower = y - x + rx;
        frontRightPower = y - x - rx;
        backRightPower = y + x - rx;


        // if one of the powers is over 1 (or maxSpeedCap), divide them by the max so that all motor powers stay the same ratio
        // (so that they're not over 1 or the maxSpeedCap)
        if (Math.abs(frontLeftPower) > maxSpeedCap || Math.abs(backLeftPower) > maxSpeedCap ||
                Math.abs(frontRightPower) > maxSpeedCap || Math.abs(backRightPower) > maxSpeedCap ) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // Divide everything by max
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower * speedMultiplier);
        backLeft.setPower(backLeftPower * speedMultiplier);
        frontRight.setPower(frontRightPower * speedMultiplier);
        backRight.setPower(backRightPower * speedMultiplier);
    }

    public void drive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, true);
    }

    /**
     * Cap the power given to the drivetrain
     * @param cap a double from 0 to 1
     */
    public void setSpeedCap(double cap) {
        if(cap > 1) {
            System.out.println("WARNING: Cannot set drivetrain speed cap over 1. Cap has been automatically set to 1");
        }
        maxSpeedCap = Range.clip(Math.abs(cap), 0, 1);
    }

    /**
     * Multiply the speed of all the motors (this is applied after the speed cap is applied)
     * @param multiplier a double from 0 to 1
     */
    public void setSpeedMultiplier(double multiplier) {
        if(multiplier > 1) {
            System.out.println("WARNING: Cannot set drivetrain speed multiplier over 1. Multiplier has been automatically set to 1");
        }
        speedMultiplier = Range.clip(Math.abs(multiplier), 0, 1);
    }

    /**
     *
     * @return double - current encoder position of frontRight motor
     */
    public int getFrontRightPosition() {
        return frontRight.getCurrentPosition();
    }

    /**
     *
     * @return double - current encoder position of front left motor
     */
    public int getFrontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }

    /**
     *
     * @return double - current encoder position of back left motor
     */
    public int getBackLeftPosition() {
        return backLeft.getCurrentPosition();
    }

    /**
     *
     * @return double - current encoder position of back right motor
     */
    public int getBackRightPosition() {
        return backRight.getCurrentPosition();
    }
}
