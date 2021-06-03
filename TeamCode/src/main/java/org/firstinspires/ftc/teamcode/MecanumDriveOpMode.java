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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.Odometry;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.SimpleOdometry;

@TeleOp(name="Test: Mecanum Drive Op Mode", group="Iterative Opmode")
public class MecanumDriveOpMode extends Constants
{
    private ElapsedTime runtime = new ElapsedTime();

    private MecanumDrive drivetrain;
    private SimpleOdometry simpleOdometry;
    private Odometry odometry;
    private Gyro gyro;

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
    public void init() {
        telemetry.addData("Status", "Initialized");

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

        gyro = new Gyro(imu);

        drivetrain = new MecanumDrive(frontRight, frontLeft, backRight, backLeft, gyro);

        simpleOdometry = new SimpleOdometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);
        odometry = new Odometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);

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

        if(gamepad1.a){
            simpleOdometry.resetPose();
        }

        double strafe = gamepad1.left_stick_y;
        double forward = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        drivetrain.drive(forward, strafe, turn);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Simple Odometry", "X Value (in): " + simpleOdometry.getCurrentVector2D().getX());
        telemetry.addData("Simple Odometry", "Y Value (in): " + simpleOdometry.getCurrentVector2D().getY());

        telemetry.addData("Odometry", "X: " + odometry.getCurrentPose().getX());
        telemetry.addData("Odometry", "Y: " + odometry.getCurrentPose().getY());
        telemetry.addData("Odometry", "Angle: " + odometry.getCurrentPose().getRotation());

        telemetry.addData("LeftEncoder", "Val: " + leftOdometer.getCurrentPosition());
        telemetry.addData("RightEncoder", "Val: " + rightOdometer.getCurrentPosition());
        telemetry.addData("backEncoder", "Val: " + backOdometer.getCurrentPosition());

        simpleOdometry.update();
        odometry.update();

    }

    @Override
    public void stop() {
    }

}
