/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.Odometry;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.SimpleOdometry;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Grabber;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Shooter;

@Autonomous(name="Main: Autonomous", group="Autonomous")
public class MainAuto extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor leftOdometer, rightOdometer, backOdometer;

    private DcMotor intakeMotor, armMotor;
    private DcMotorEx shooterMotor;
    private Servo indexer, grabberServo;

    private BNO055IMU imu;

    private MecanumDrive drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Grabber grabber;

    private Gyro gyro;

    private Odometry odometry;
    private SimpleOdometry simpleOdometry;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRight = hardwareMap.get(DcMotor.class, "backRightDrive");

        leftOdometer = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        rightOdometer = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backOdometer = hardwareMap.get(DcMotor.class, "backLeftDrive");

        // Reset encoders
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set direction of the motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        indexer = hardwareMap.get(Servo.class, "indexerServo");
        grabberServo = hardwareMap.get(Servo.class, "grabberServo");

        gyro = new Gyro(imu);
        drivetrain = new MecanumDrive(frontRight, frontLeft, backRight, backLeft, gyro);
        intake = new Intake(intakeMotor);
        shooter = new Shooter(shooterMotor, indexer);
        grabber = new Grabber(armMotor, grabberServo);


        odometry = new Odometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);
        simpleOdometry = new SimpleOdometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        odometry.update();
        simpleOdometry.update();

        shooter.setIndexer(-90);

        // Driving
        if(simpleOdometry.getCurrentVector2D().getX() < 60) {
            drivetrain.driveStraight(.7, 0);
        }
        else if(simpleOdometry.getCurrentVector2D().getX() > 60 && gyro.getAngle() > -90) {
            drivetrain.drive(0, 0, -.5);
        }
        else if(simpleOdometry.getCurrentVector2D().getX() >= 60 && gyro.getAngle() <= -90) {
            double currentTime = getRuntime();
            shooter.on();
            if(currentTime == currentTime + .5) {
                shooter.setIndexer(90);
            }
            else if(currentTime == currentTime + 1) {
                shooter.setIndexer(-90);
            }
            else if(currentTime == currentTime + 1.5) {
                shooter.setIndexer(90);
            }
            else if(currentTime == currentTime + 2) {
                shooter.setIndexer(-90);
            }
            else if(currentTime == currentTime + 2.5) {
                shooter.setIndexer(90);
            }
            else if(currentTime == currentTime + 3) {
                shooter.setIndexer(-90);
            }

        }
        else {
            drivetrain.drive(0, 0, 0);
            shooter.off();
        }


        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Simple Odometry", "X Value (in): " + simpleOdometry.getCurrentVector2D().getX());
        telemetry.addData("Simple Odometry", "Y Value (in): " + simpleOdometry.getCurrentVector2D().getY());

        telemetry.addData("Odometry", "X: " + odometry.getCurrentPose().getX());
        telemetry.addData("Odometry", "Y: " + odometry.getCurrentPose().getY());
        telemetry.addData("Odometry", "Angle: " + odometry.getCurrentPose().getRotation());

        telemetry.addData("LeftEncoder", "Val: " + leftOdometer.getCurrentPosition());
        telemetry.addData("RightEncoder", "Val: " + rightOdometer.getCurrentPosition());
        telemetry.addData("backEncoder", "Val: " + backOdometer.getCurrentPosition());
    }

    @Override
    public void stop() {
    }

    public void forwardInches(double power, double inches) {
        if(simpleOdometry.getCurrentVector2D().getX() < inches) {
            drivetrain.driveStraight(power, 0);
        }
        else {
            drivetrain.drive(0, 0, 0);
        }
    }

    public void strafeInches(double power, double inches) {
        if(Math.abs(simpleOdometry.getCurrentVector2D().getY()) < inches) {
            drivetrain.driveStraight(0, power * Math.copySign(1, simpleOdometry.getCurrentVector2D().getY() * -1));
        }
        else {
            drivetrain.drive(0, 0, 0);
        }
    }

    public void turnAngle(double power, double angle) {
        double current = Math.abs(gyro.getAngle());
        double target = Math.abs(current + angle);
        if(Math.abs(gyro.getAngle()) < target) {
            drivetrain.drive(0, 0, power);
        }
        else {
            drivetrain.drive(0, 0, 0);
        }

    }

    public void autoShoot(double times) {
        shooter.on();
        for(int i = 0; i < times; i++) {
            shooter.runIndexer();
        }
        shooter.off();
    }

}
