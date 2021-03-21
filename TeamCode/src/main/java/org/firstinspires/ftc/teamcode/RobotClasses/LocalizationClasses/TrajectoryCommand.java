package org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses;

import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.util.PIDController;
import org.firstinspires.ftc.teamcode.RobotClasses.util.Pose2D;

import java.util.ArrayList;

public class TrajectoryCommand {
    private ArrayList<Pose2D> trajectory;
    private Odometry odometry;
    private MecanumDrive drivetrain;

    public TrajectoryCommand(Odometry odometry, MecanumDrive drivetrain) {
        this.odometry = odometry;
        this.drivetrain = drivetrain;
    }

    public void buildTrajectory(ArrayList<Pose2D> trajectory) {
        this.trajectory = trajectory;
    }

    public boolean atX(Pose2D targetWaypoint) {
        double currentX = odometry.getCurrentPose().getX();
        double targetX = targetWaypoint.getX();
        return currentX > targetX - 1 && currentX < targetX + 1;
    }

    public boolean atY(Pose2D targetWaypoint) {
        double currentY = odometry.getCurrentPose().getY();
        double targetY = targetWaypoint.getY();
        return currentY > targetY - 1 && currentY < targetY + 1;
    }

    public boolean atRotation(Pose2D targetWaypoint) {
        double currentRotation = odometry.getCurrentPose().getRotation();
        double targetRotation = targetWaypoint.getRotation();
        return currentRotation > targetRotation - 3 && currentRotation < targetRotation + 3;
    }

    public void goToPoint(Pose2D targetWaypoint) {
        PIDController pid = new PIDController(.01, .001, .01);
        pid.setOutputRange(-.4, .4);
        double forward, strafe, rotate;

        if(!atX(targetWaypoint)) {
            forward = -pid.calculate(odometry.getCurrentPose().getX(), targetWaypoint.getX());
        }
        else {
            forward = 0;
        }

        if(!atY(targetWaypoint)) {
            strafe = -pid.calculate(odometry.getCurrentPose().getY(), targetWaypoint.getY());
        }
        else {
            strafe = 0;
        }

        if(!atRotation(targetWaypoint)) {
            rotate = -pid.calculate(odometry.getCurrentPose().getRotation(), targetWaypoint.getRotation()) * .7;
        }
        else {
            rotate = 0;
        }

        drivetrain.drive(forward, strafe, rotate);


    }

}
