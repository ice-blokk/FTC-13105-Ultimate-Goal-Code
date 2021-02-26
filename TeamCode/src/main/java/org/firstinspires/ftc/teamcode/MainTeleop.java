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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.Shooter;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.SimpleOdometry;

@TeleOp(name="Main: Teleop", group="Iterative Opmode")
@Disabled
public class MainTeleop extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor intakeMotor;
    private DcMotorEx shooterMotor;
    private Servo indexer;

    private BNO055IMU imu;

    private MecanumDrive drivetrain;
    private Intake intake;
    private Shooter shooter;

    private SimpleOdometry odometry;
    private Gyro gyro;

    boolean x_boolean, y_boolean;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRight = hardwareMap.get(DcMotor.class, "backRightDrive");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        indexer = hardwareMap.get(Servo.class, "indexerServo");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // idk if this actually needs to be here
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        drivetrain = new MecanumDrive(frontRight, frontLeft, backRight, backLeft, gyro);
        intake = new Intake(intakeMotor);
        shooter = new Shooter(shooterMotor, indexer);
        gyro = new Gyro(imu);

        // Toggle Booleans
        x_boolean = false;
        y_boolean = false;


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

        // Driving
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        drivetrain.drive(forward, strafe, turn);

        // Toggle booleans
        if(gamepad1.x) {
            x_boolean = !x_boolean;
        }
        if(gamepad1.y) {
            y_boolean = !y_boolean;
        }

        // Intake On/Off
        if(x_boolean) {
            intake.on();
        }
        else if(gamepad1.b) {
            intake.reverse();
        }
        else {
            intake.off();
        }

        // Shooter On/Off
        if(y_boolean) {
            shooter.on();
        }
        else {
            shooter.off();
        }

        // Indexer
        if(gamepad1.a) {
            shooter.runIndexer();
        }


        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {
    }

}
