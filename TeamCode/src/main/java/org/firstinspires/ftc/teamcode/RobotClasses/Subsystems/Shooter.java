package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class for a shooter with a servo.
 *
 * NOTE: For the HD Hex Motor that we're using for the shooter, the max RPM is 6000 (with a 1:1 gear ratio)
 */
public class Shooter {
    private DcMotorEx shooterMotor;
    private Servo indexer;

    private double velocity;

    public Shooter(DcMotorEx shooterMotor, Servo indexer) {
        this.shooterMotor = shooterMotor;
        this.indexer = indexer;

        velocity = 6000;
    }

    public void on() {
        shooterMotor.setVelocity(velocity);
    }

    public void off() {
        shooterMotor.setVelocity(0);
    }

    public void setDefaultVelocity(double velocity) {
        velocity = this.velocity;
    }

    public void set(double p) {
        shooterMotor.setPower(p);
    }

    public void setVelocity(double v) {
        shooterMotor.setVelocity(v);
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        shooterMotor.setVelocityPIDFCoefficients(p, i , d, f);
    }

    public DcMotorEx getMotor() {
        return shooterMotor;
    }

    public void runIndexer() {
        indexer.setPosition(180);
        indexer.setPosition(0);
    }
}
