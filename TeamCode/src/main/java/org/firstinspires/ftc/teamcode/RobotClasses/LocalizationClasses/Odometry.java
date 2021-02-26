package org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotClasses.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrive;

/**
 * Localization class for 3 wheel Odometry. This class is based on Centennial High School FTC (Team VIRUS #9866).
 * <a href = https://github.com/Centennial-FTC-Robotics/Virus2019-2020/blob/master/TeamCode/src/main/java/org/virus/agobot/Odometry.java>
 *     This was the code I used for the math. </a>
 * A derivation of the math used can be found <a href = https://drive.google.com/drive/u/1/folders/1b67UYvB0yLR7qvaCotbuCF70fAjkADr4>here.</a>
 */
public class Odometry extends SimpleOdometry {
    MecanumDrive drivetrain;
    Gyro gyro;

    Pose2D currentPose;

    DcMotorEx leftOdometer, rightOdometer, backOdometer;

    double deltaLeftEncoder, deltaRightEncoder, deltaBackEncoder;
    double leftEncoderPrev, rightEncoderPrev, backEncoderPrev;
    double deltaHeading;
    double deltaX, deltaY;

    double heading = 0, startHeading = 0, headingCorrection = 0;

    final double trackwidth = 16.0; //in inches
    final double length = 17.0; // in inches

    final double ENCODER_COUNTS_PER_INCH = CPR / (wheelDiameter * Math.PI);
    // TODO: change these numbers
    final double RADIUS = trackwidth / 2.0; // in inches, the distance from the middle of the robot to the left/right odometer
    final double BACK_RADIUS = 0; //in inches, the distance from the back odometer to the left/right odometer

    Vector2D robotCentricDelta;

    public Odometry(MecanumDrive drivetrain, Gyro gyro, DcMotorEx leftOdometer, DcMotorEx rightOdometer, DcMotorEx backOdometer) {
        super(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);

        currentPose = new Pose2D(0, 0, gyro.getAngle());
    }

    public void resetAllEncoders() {
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double normalizeRadians(double angleRadians) {
        while(angleRadians > 2 * Math.PI) {
            angleRadians -= 2 * Math.PI;
        }
        while(angleRadians < 0.0) {
            angleRadians += 2 * Math.PI;
        }
        return angleRadians;
    }

    public double getLeftPosition() {
        return leftOdometer.getCurrentPosition();
    }

    public double getRightPosition() {
        return rightOdometer.getCurrentPosition() * -1;
    }

    public double getBackPosition() {
        return backOdometer.getCurrentPosition() * -1;
    }

    /**
     * Works much like the getEncoderDistance() method in the SimpleOdometry class
     * but accepts a double rather than an encoder
     * @param count encoder value
     * @return distance the encoder wheel has traveled in inches
     */
    public double countToInch(double count) {
        return (count / CPR) * Math.PI * wheelDiameter;
    }

    public Pose2D getCurrentPose() {
        return currentPose;
    }

    public Vector2D getRobotCentricDelta() {
        return robotCentricDelta;
    }

    public void update() {
        deltaLeftEncoder = getLeftPosition() - leftEncoderPrev;
        deltaRightEncoder = getRightPosition() - rightEncoderPrev;
        deltaBackEncoder = getBackPosition() - backEncoderPrev;

        leftEncoderPrev = getLeftPosition();
        rightEncoderPrev = getRightPosition();
        backEncoderPrev = getBackPosition();

        deltaHeading = (deltaRightEncoder - deltaLeftEncoder) / (2.0 * RADIUS * ENCODER_COUNTS_PER_INCH); // in radians
        heading = normalizeRadians((getRightPosition() - getLeftPosition())) / (2.0 * RADIUS * ENCODER_COUNTS_PER_INCH) + startHeading + headingCorrection;

        if(deltaHeading == 0) {
            deltaX = deltaBackEncoder;
            deltaY = (deltaBackEncoder + deltaRightEncoder) / 2.0;
        }
        else {
            double turnRadius = 2.0 * RADIUS * ENCODER_COUNTS_PER_INCH * (deltaLeftEncoder + deltaRightEncoder) / (deltaRightEncoder - deltaLeftEncoder);
            double strafeRadius = deltaBackEncoder / deltaHeading - BACK_RADIUS * ENCODER_COUNTS_PER_INCH;

            deltaX = turnRadius * (Math.cos(deltaHeading) - 1) + strafeRadius * Math.sin(deltaHeading);
            deltaY = turnRadius * Math.sin(deltaHeading) + strafeRadius * (1 - Math.cos(deltaHeading));
        }

        robotCentricDelta = new Vector2D(countToInch(deltaX), countToInch(deltaY));
        currentPose = new Pose2D(robotCentricDelta, gyro.getAngle());
    }
}
