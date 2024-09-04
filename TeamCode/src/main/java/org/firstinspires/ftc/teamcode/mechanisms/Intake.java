package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseRobot;


public class Intake {
    public static double intakeSpeed = 0.6;
    public static boolean isReversed = false;
    private static boolean intakeOn;
    private final BaseRobot baseRobot;
    private final DcMotor intakeMotor;

    public Intake(BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        intakeMotor = baseRobot.hardwareMap.get(DcMotor.class, "intake");
    }

    public void forward() {
        intakeOn = true;
        isReversed = false;
        update();
    }

    public void backward() {
        intakeOn = true;
        isReversed = true;
        update();
    }

    /**
     * Switch the intake state from on to off, or vice versa.
     */
    public void switchIntake() {
        intakeOn = !intakeOn;
        update();
    }

    /**
     * Update the intake state (on/off).
     */
    public void update() {
        update(true);
    }

    /**
     * Update the intake state (on/off).
     *
     * @param on true to turn on the intake, false to turn it off.
     */
    public void update(boolean on) {
        double absoluteIntakeSpeed = isReversed ? intakeSpeed : -intakeSpeed;
        intakeOn = on;
        intakeMotor.setPower(intakeOn ? absoluteIntakeSpeed : 0);
    }

    /**
     * Start the intake.
     */
    public void start() {
        update();
    }

    /**
     * Stop the intake.
     */
    public void stop() {
        update(false);
    }

    /**
     * Change the intake speed by the given amount.
     *
     * @param change Amount by which to change the intake speed.
     */
    public void changeSpeed(double change) {
        double newSpeed = intakeSpeed + change;
        newSpeed = Math.max(Math.min(newSpeed, -1), 1);
        intakeSpeed = newSpeed;
        update();
    }

    /**
     * Set the intake speed to a specific value.
     *
     * @param newSpeed The new speed of the intake.
     */
    public void setSpeed(double newSpeed) {
        intakeSpeed = Math.max(Math.min(newSpeed, -1), 1);
        update();
    }

    /**
     * Reverse the intake mechanism.
     */
    public void reverse() {
        isReversed = !isReversed;
        update();
    }
}

