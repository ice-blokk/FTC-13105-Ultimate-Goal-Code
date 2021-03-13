package org.firstinspires.ftc.teamcode.RobotClasses.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Class for the Intake
 */
public class Intake {
    private DcMotor intakeMotor;
    private double power;

    public Intake(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;

        power = -1;
    }

    /**
     * Turns on the intake motor to intake rings
     */
    public void on() {
        intakeMotor.setPower(power);
    }

    /**
     * Turns off the intake motor
     */
    public void off() {
        intakeMotor.setPower(0);
    }

    /**
     * Turns on the intake motor to eject rings
     */
    public void reverse() {
        intakeMotor.setPower(-power);
    }

    /**
     * Sets motor power
     * @param power power which the motor will run (both forwardsd and reverse)
     */
    public void setMotorPower(double power) {
        power = Range.clip(Math.abs(power), 0, 1);
    }
}
