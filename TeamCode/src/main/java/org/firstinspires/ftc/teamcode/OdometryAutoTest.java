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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.SimpleOdometry;
import org.firstinspires.ftc.teamcode.RobotClasses.util.Vector2D;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name="Test: Odometry Auto Test", group="Iterative Opmode")
public class OdometryAutoTest extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotorEx leftOdometer, rightOdometer, backOdometer;

    private BNO055IMU imu;
    private Gyro gyro;

    private MecanumDrive drivetrain;
    private SimpleOdometry simpleOdometry;

    private ArrayList<Vector2D> trajectory;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRight = hardwareMap.get(DcMotor.class, "backRightDrive");

        // idk if this actually needs to be here

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        leftOdometer = hardwareMap.get(DcMotorEx.class, "leftOdometer");
        rightOdometer = hardwareMap.get(DcMotorEx.class, "rightOdometer");
        backOdometer = hardwareMap.get(DcMotorEx.class, "backOdometer");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        gyro = new Gyro(imu);

        drivetrain = new MecanumDrive(frontRight, frontLeft, backRight, backLeft, gyro);
        simpleOdometry = new SimpleOdometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);

        trajectory = new ArrayList<>(Arrays.asList(
                new Vector2D(10, 10),
                new Vector2D(20, -10),
                new Vector2D(30, 0)
        ));


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

        simpleOdometry.update();

        simpleOdometry.followTrajectory(trajectory, .3, .3);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Odometry", "X Value (in): " + simpleOdometry.getCurrentVector2D().getX());
        telemetry.addData("Odometry", "Y Value (in): " + simpleOdometry.getCurrentVector2D().getY());
    }

    @Override
    public void stop() {
    }

}
