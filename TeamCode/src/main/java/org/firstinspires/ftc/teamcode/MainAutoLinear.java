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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
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

import java.util.List;

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

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AeO4IGj/////AAABmU63rhEa2UHcp6WYD9aPftR0Ng+l2Y1rTyE504+3KOuaDgvQMCW9M+GdjYidkvXsch5FEbYgsCtnACgS/CFcN6ZcJuyGALGfShSJ7+lZC5JOO4muO9G8GtoF+29tsSFLzUloVHHnC7dTpjxkOdJMfBiJWd5BlwVk08ESHIFRg4XoyCTrgkUAzljSW6u3b6uhW+IrtYvcocQMJude0+a8kckI5iN25AabaOcj108frbkVki0uTZzehjG4u2Ve2eUvNY7q7hqd3QtA0gB58K3wYnxpbWkbA/QActi/+ogNBdJb8bNUv1VGi32QDJrNabymUebURoM5fmq91Is9nUltgY5RifOhe8U3b/laxAX+5ZQB";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private String caseString;


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

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }
        caseString = null;

        waitForStart();



        runtime.reset();

        //** MAIN LOOP **//
        while (opModeIsActive()) {

            odometry.update();
            simpleOdometry.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Odometry", "X Value (in): " + odometry.getCurrentVector2D().getX());
            telemetry.addData("Odometry", "Y Value (in): " + odometry.getCurrentVector2D().getY());

            telemetry.addData("Angle", gyro.getAngle());
            telemetry.update();

            // Detect Rings
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        caseString = recognition.getLabel();
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    telemetry.update();
                }
            }
            telemetry.addData("caseString", caseString);
            telemetry.update();

            if(caseString != null) {
                if(caseString.equals("Single")) {
                    //single();
                }
                else if(caseString.equals("Quad")) {
                    // quad();
                }
            }
            else {
                quad();
            }

            /*
            sleepMil(10000);
            stop();


             */





        }

        if (tfod != null) {
            tfod.shutdown();
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

    public void armDown() {
        while(armMotor.getCurrentPosition() < 1500) {
            grabber.down();
        }
        grabber.off();
    }

    public void armUp() {
        while(armMotor.getCurrentPosition() > 10) {
            grabber.up();
        }
        grabber.off();
    }

    public void driveWaypoint(Pose2D targetWaypoint) {
        odometry.update();
        traj.setDrivePower(.6);
        traj.setRotatePower(.3);
        while(!traj.atWaypoint(targetWaypoint)) {
            odometry.update();
            traj.goToPoint(targetWaypoint);
        }
        drivetrain.drive(0, 0, 0);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // the line above gives an error in android studio but builds fine in onbot java
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void single() {
        shooter.setIndexer(-90);

        driveWaypoint(new Pose2D(0, 15, 0)); // move to the left of the rings
        sleepMil(250);
        driveWaypoint(new Pose2D(45, 15 ,0)); // go to wobble square
        sleepMil(250);
        armDown();
        grabber.openGrabber();
        sleepMil(250);

        driveWaypoint(new Pose2D(0, 15, 0)); // go to the left of the rings
        sleepMil(250);
        driveWaypoint(new Pose2D(0, -10, 0)); // go to wobble goal
        grabber.closeGrabber();
        sleepMil(350);
        driveWaypoint(new Pose2D(0, 15, 0)); // move to the left of the rings
        driveWaypoint(new Pose2D(60, 15 ,0)); // go to wobble square
        grabber.openGrabber();
        sleepMil(250);

        driveWaypoint(new Pose2D(40, 5, 90)); // go to shooter position
        shooter.on(); // start shooter sequence
        sleepMil(750);
        shooter.setIndexer(90); //shoot
        sleepMil(500);
        shooter.setIndexer(-90); // reset
        sleepMil(500);
        shooter.setIndexer(90); // shoot
        sleepMil(500);
        shooter.setIndexer(-90); // reset
        sleepMil(500);
        shooter.setIndexer(90); // shoot
        shooter.off();

        driveWaypoint(new Pose2D(30, 10, 90)); // drive up to line
        grabber.closeGrabber();
        armUp();

        stop();
    }

    private void quad() {
        shooter.setIndexer(-90);

        driveWaypoint(new Pose2D(0, 15, 0)); // move to the left of the rings
        sleepMil(250);
        driveWaypoint(new Pose2D(60, 15 ,0));
        driveWaypoint(new Pose2D(80, 0 ,0)); // go to wobble square
        sleepMil(250);
        armDown();
        grabber.openGrabber();
        sleepMil(250);

        driveWaypoint(new Pose2D(0, 15, 0)); // go to the left of the rings
        sleepMil(250);
        driveWaypoint(new Pose2D(0, -10, 0)); // go to wobble goal
        grabber.closeGrabber();
        sleepMil(350);
        driveWaypoint(new Pose2D(0, 15, 0)); // move to the left of the rings
        driveWaypoint(new Pose2D(60, 15 ,0));
        driveWaypoint(new Pose2D(80, 0 ,0)); // go to wobble square
        grabber.openGrabber();
        sleepMil(250);

        driveWaypoint(new Pose2D(40, 5, 90)); // go to shooter position
        shooter.on(); // start shooter sequence
        sleepMil(750);
        shooter.setIndexer(90); //shoot
        sleepMil(500);
        shooter.setIndexer(-90); // reset
        sleepMil(500);
        shooter.setIndexer(90); // shoot
        sleepMil(500);
        shooter.setIndexer(-90); // reset
        sleepMil(500);
        shooter.setIndexer(90); // shoot
        shooter.off();

        driveWaypoint(new Pose2D(30, 10, 90)); // drive up to line
        grabber.closeGrabber();
        armUp();

        stop();

    }

    private void noRings() {
        shooter.setIndexer(-90);

        driveWaypoint(new Pose2D(30, 0 ,0)); // go to wobble square
        sleepMil(250);
        armDown();
        grabber.openGrabber();
        sleepMil(250);
        driveWaypoint(new Pose2D(0, 0, 0)); // go to origin
        sleepMil(250);
        driveWaypoint(new Pose2D(0, -10, 0)); // go to wobble goal
        grabber.closeGrabber();
        sleepMil(250);
        driveWaypoint(new Pose2D(50, 0, 0)); // drop off wobble goal
        grabber.openGrabber();
        sleepMil(250);

        driveWaypoint(new Pose2D(40, 5, 90)); // go to shooter position
        shooter.on(); // start shooter sequence
        sleepMil(750);
        shooter.setIndexer(90); //shoot
        sleepMil(500);
        shooter.setIndexer(-90); // reset
        sleepMil(500);
        shooter.setIndexer(90); // shoot
        sleepMil(500);
        shooter.setIndexer(-90); // reset
        sleepMil(500);
        shooter.setIndexer(90); // shoot
        shooter.off();

        driveWaypoint(new Pose2D(60, 10, 90)); // drive up to line
        grabber.closeGrabber();
        armUp();

        stop();



    }

}// end of class
