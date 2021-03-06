package org.firstinspires.ftc.teamcode.RobotClasses.util;

/**
 * A class for (x, y) coordinates. Any unit can be used to represent x and y but
 * inches are used in the this projects Odometry classes
 */
public class Vector2D {
    private double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public String toString() {
        return "X: " + x + "/ Y: " + y;
    }

}
