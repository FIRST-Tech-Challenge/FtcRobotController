package org.firstinspires.ftc.teamcode.Tests;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

/**
 * William
 * program to effectively be able to tune the constants of a servo from dashboard, abstract so new devices can just extend it
 */

public abstract class RFServoTest extends LinearOpMode {
    RFServo testServo;
    BasicRobot robot;
    double lastTime = 0;
    boolean flipped = false;
    double FLIP_TIME;

    double SERVO_LOWER_LIMIT;
    double SERVO_UPPER_LIMIT;


    /**
     * Initalizes RFServoTest class
     * @param p_testServoName name of RFServo that needs to be tested
     * @param p_FLIP_TIME time it takes for servo to flip from lower limit to upper limit
     * @param p_SERVO_LOWER_LIMIT lower limit of servo
     * @param p_SERVO_UPPER_LIMIT upper limit of servo
     */
    public void initialize(String p_testServoName, double p_FLIP_TIME, double p_SERVO_LOWER_LIMIT,
                       double p_SERVO_UPPER_LIMIT) {
        robot = new BasicRobot(this, false);
        testServo = new RFServo(p_testServoName, p_SERVO_UPPER_LIMIT);
        FLIP_TIME = p_FLIP_TIME;
        SERVO_LOWER_LIMIT = p_SERVO_LOWER_LIMIT;
        SERVO_UPPER_LIMIT = p_SERVO_UPPER_LIMIT;
    }

    /**
     * Continuously sets servo to lower limit and upper limit.
     * Logs that the servo has been flipped to max positions.
     * Logs to RFServo & general logs.
     * Logs to second finest level.
     * Does not update a state machine.
     */
    public void autoLoop() {
        if (time - lastTime > FLIP_TIME) {
            if (flipped) {
                testServo.setPosition(SERVO_LOWER_LIMIT);
                LOGGER.log(RFLogger.Severity.INFO, "RFServoTest.autoLoop(): flipped servo to position: " + SERVO_LOWER_LIMIT);
                flipped = false;
            } else {
                testServo.setPosition(SERVO_UPPER_LIMIT);
                LOGGER.log(RFLogger.Severity.INFO, "RFServoTest.autoLoop(): flipped servo to position: " + SERVO_UPPER_LIMIT);
                flipped = true;
            }
            lastTime = time;
        }
        robot.update();
    }
}
