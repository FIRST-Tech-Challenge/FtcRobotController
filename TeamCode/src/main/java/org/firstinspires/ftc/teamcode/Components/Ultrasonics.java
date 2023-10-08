package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFUltrasonic;

/**
 * William
 */
public class Ultrasonics {
    RFUltrasonic backLeft, backRight, frontLeft, frontRight;

    private double lastFlipTime = 0;

    /**
     * Constructor
     */
    public Ultrasonics(){
        backLeft = new RFUltrasonic("backLeftUltra");
        backRight = new RFUltrasonic("backRightUltra");
        frontLeft = new RFUltrasonic("frontLeftUltra");
        frontRight = new RFUltrasonic("frontRightUltra");
    }
    /**
     * Checks if there is a robot in our backstage near the backdrop.
     * Logs whether it found a robot or not.
     * Logs to RFUltrasonic & general logs.
     * Logs to second finest level.
     * Does not update a state machine.
     */
    public boolean checkAlliance() {
        return frontLeft.check() && frontRight.check();
    }
    /**
     * Checks if there is a robot near or coming to the opponent's pixel stack(s).
     * Logs whether it found a robot or not.
     * Logs to RFUltrasonic & general logs.
     * Logs to second finest level.
     * Does not update a state machine.
     */
    public boolean checkOpp() {
        return frontRight.check() && backRight.check();
    }
    /**
     * Updates if the pins on the ultrasonics have been flipped.
     * Logs the status of the pins.
     * Logs to RFUltrasonics & general logs.
     * Logs to second finest level.
     * Does not update a state machine.
     */
    public void update(){
        if (op.getRuntime() - lastFlipTime > 0.1) {
            frontLeft.flipPin();
            frontRight.flipPin();
            backLeft.flipPin();
            backRight.flipPin();
        }
    }

}
