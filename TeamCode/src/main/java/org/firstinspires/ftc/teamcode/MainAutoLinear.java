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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.Odometry;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.SimpleOdometry;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Grabber;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotClasses.util.Pose2D;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Main: Auto Linear", group="Autonomous")
public class MainAutoLinear extends LinearOpMode {

    // Declare OpMode members.
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

    private TrajectoryCommand traj;

    @Override
    public void runOpMode() {
        //** INIT **//
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

        traj = new TrajectoryCommand(odometry, drivetrain);

        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();

        //** MAIN LOOP **//
        while (opModeIsActive()) {

            odometry.update();
            simpleOdometry.update();
            /*
            shooter.setIndexer(-90);
            boolean finished = false;
            // Driving
            if(simpleOdometry.getCurrentVector2D().getX() < 40) {
                drivetrain.drive(-.6, 0, 0);
            }
            else if(simpleOdometry.getCurrentVector2D().getX() > 40 && gyro.getAngle() < 70) {
                drivetrain.drive(0, 0, -.5);
            }
            else if(simpleOdometry.getCurrentVector2D().getX() >= 40 && gyro.getAngle() >= 70 && finished == false) {
                drivetrain.drive(0, 0, 0);
                shooter.on();
                sleepMil(1000);
                shooter.setIndexer(90);
                sleepMil(1500);
                shooter.setIndexer(-90);
                sleepMil(1500);
                shooter.setIndexer(90);
                sleepMil(1500);
                shooter.setIndexer(-90);
                sleepMil(1500);
                shooter.setIndexer(90);
                shooter.off();
                sleepMil(1500);
                drivetrain.drive(-.5, 0, 0);
                sleepMil(1200);
                drivetrain.drive(0, 0, 0);
                finished = true;
                stop();

            }
            else {
                drivetrain.drive(0, 0, 0);
                shooter.off();
            }

             */

            shooter.setIndexer(-90);
            driveWaypoint(new Pose2D(10, 10, 0));
            sleepMil(3000);
            driveWaypoint(new Pose2D(20, -10, 90));
            /*
            shooter.on();
            sleepMil(1000);
            shooter.setIndexer(90);
            sleepMil(1500);
            shooter.setIndexer(-90);
            sleepMil(1500);
            shooter.setIndexer(90);
            sleepMil(1500);
            shooter.setIndexer(-90);
            sleepMil(1500);
            shooter.setIndexer(90);
            shooter.off();

             */
            stop();





            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Simple Odometry", "X Value (in): " + simpleOdometry.getCurrentVector2D().getX());
            telemetry.addData("Simple Odometry", "Y Value (in): " + simpleOdometry.getCurrentVector2D().getY());

            telemetry.addData("Angle", gyro.getAngle());
            telemetry.update();
        }
    } // end of active op mode


    //** METHODS **//


    // Wait command (accepts time in milliseconds)
    public static void sleepMil(long sleepTime)
    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0)
        {
            try
            {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
            }
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    }

    public void fowardInches(double power, double inches) {
        double currentX = simpleOdometry.getCurrentVector2D().getX();
        if(inches > 0) {
            if (simpleOdometry.getCurrentVector2D().getX() < currentX + inches) {
                drivetrain.drive(power, 0, 0, false);
            }
            else {
                drivetrain.drive(0, 0, 0);
            }
        }
        else {
            if(simpleOdometry.getCurrentVector2D().getX() > currentX - inches) {
                drivetrain.drive(power, 0, 0, false);
            }
            else {
                drivetrain.drive(0, 0, 0);
            }
        }
    }

    public void strafeInches(double power, double inches) {
        double currentY = simpleOdometry.getCurrentVector2D().getY();
        if(inches > 0) {
            if(simpleOdometry.getCurrentVector2D().getY() < currentY + inches) {
                drivetrain.drive(0, power, 0, false);
            }
            else {
                drivetrain.drive(0, 0, 0);
            }
        }
        else {
            if(simpleOdometry.getCurrentVector2D().getY() > currentY - inches) {
                drivetrain.drive(0, power, 0, false);
            }
            else {
                drivetrain.drive(0, 0, 0);
            }
        }
    }

    public void turnAngle(double power, double angle) {
        double currentAngle = gyro.getAngle();
        if(angle > 0) {
            if(gyro.getAngle() < currentAngle + angle) {
                drivetrain.drive(0, 0, power);
            }
            else {
                drivetrain.drive(0, 0, 0);
            }
        }
        else {
            if(gyro.getAngle() > currentAngle - angle) {
                drivetrain.drive(0, 0, power);
            }
            else{
                drivetrain.drive(0, 0, 0);
            }
        }
    }

    public void driveWaypoint(Pose2D targetWaypoint) {
        while(!traj.atX(targetWaypoint) || !traj.atY(targetWaypoint) || !traj.atRotation(targetWaypoint)) {
            traj.goToPoint(targetWaypoint);
        }
            drivetrain.drive(0, 0, 0);
    }

}// end of class
