package org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrive;
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
        return currentX > targetX - .5 && currentX < targetX + .5;
    }

    public boolean atY(Pose2D targetWaypoint) {
        double currentY = odometry.getCurrentPose().getY();
        double targetY = targetWaypoint.getY();
        return currentY > targetY - .5 && currentY < targetY + .5;
    }

    public boolean atRotation(Pose2D targetWaypoint) {
        double currentRotation = odometry.getCurrentPose().getRotation();
        double targetRotation = targetWaypoint.getRotation();
        return currentRotation > targetRotation - 1 && currentRotation < targetRotation + 1;
    }

    public void followTrajectory() {

    }

}
