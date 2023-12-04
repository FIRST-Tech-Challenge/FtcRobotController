package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.Line;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFUltrasonic;

/**
 * William
 */
public class Ultrasonics {
    RFUltrasonic backLeft, backRight, frontLeft, frontRight;

    private double lastFlipTime = 0;

    Line allianceLine = new Line(1,0,56, new Vector2d(56,-54), new Vector2d(56,-18));

    Line oppLine = new Line(1, 0, 56, new Vector2d(24, 0),
            new Vector2d(0, 0));

    /**
     * Constructor
     */
    public Ultrasonics(){
        backLeft = new RFUltrasonic("backLeftUltraAnalog", "backLeftUltraLED");
//        backRight = new RFUltrasonic("backRightUltra");
//        frontLeft = new RFUltrasonic("frontLeftUltra");
//        frontRight = new RFUltrasonic("frontRightUltra");
    }
    /**
     * Checks if there is a robot in our backstage near the backdrop.
     * Logs whether it found a robot or not.
     * Logs to RFUltrasonic & general logs.
     * Logs to second finest level.
     * Does not update a state machine.
     */
    public boolean checkAlliance() {
        backLeft.setLine(allianceLine);
        op.telemetry.addData("dist", backLeft.getDist());
//        frontRight.setLine(allianceLine);
        return backLeft.isDetected();
//                && frontRight.isDetected();
    }
    /**
     * Checks if there is a robot near or coming to the opponent's pixel stack(s).
     * Logs whether it found a robot or not.
     * Logs to RFUltrasonic & general logs.
     * Logs to second finest level.
     * Does not update a state machine.
     */
    public boolean checkOpp() {
        backLeft.setLine(oppLine);
//        frontRight.setLine(oppLine);
        return backLeft.isDetected();
//        frontRight.isDetected();
    }
    /**
     * Updates if the pins on the ultrasonics have been flipped.
     * Logs the status of the pins.
     * Logs to RFUltrasonics & general logs.
     * Logs to second finest level.
     * Does not update a state machine.
     */
    public void update() {
        backLeft.check();
    }
}
