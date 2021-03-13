package org.firstinspires.ftc.teamcode.RobotClasses.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * A wrapper class for the BNO055IMU
 */
public class Gyro {
    private BNO055IMU imu;

    Orientation angle;

    public Gyro(BNO055IMU imu) {
        this.imu = imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /**
     *
     * @return the imu's Z angle in degrees
     */
    public double getAngle() {
        imu.getPosition();
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
    }

    /**
     *
     * @return the imu's Z angle in degrees in the range 0 to 360
     */
    public double getClampedAngle() {
        double angle = getAngle();

        while(angle > 360) {
            angle -= 360;
        }
        while(angle < 0) {
            angle += 360;
        }

        return angle;
    }

}
