package org.firstinspires.ftc.teamcode.Tests;



import static org.firstinspires.ftc.teamcode.Components.RFModules.System.Logger.df;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

/**
 * William
 * program to effectively be able to tune the constants of a servo from dashboard, abstract so new devices can just extend it
 */

public abstract class RFServoTest extends LinearOpMode {
    RFServo testServo;
    double lastTime = 0;
    boolean flipped = false;
    double FLIP_TIME;

    double SERVO_LOWER_LIMIT;
    double SERVO_UPPER_LIMIT;

    /**
     * Constructs RFServoTest class
     * @param p_testServo RFServo that needs to be tested
     * @param p_FLIP_TIME time it takes for servo to flip from lower limit to upper limit
     * @param p_SERVO_LOWER_LIMIT lower limit of servo
     * @param p_SERVO_UPPER_LIMIT upper limit of servo
     */
    public RFServoTest(RFServo p_testServo, double p_FLIP_TIME, double p_SERVO_LOWER_LIMIT, double p_SERVO_UPPER_LIMIT) {
        testServo = p_testServo;
        FLIP_TIME = p_FLIP_TIME;
        SERVO_LOWER_LIMIT = p_SERVO_LOWER_LIMIT;
        SERVO_UPPER_LIMIT = p_SERVO_UPPER_LIMIT;
    }

    /**
     * Continuously sets servo to lower limit and upper limit.
     */
    public void autoLoop() {
        if (time - lastTime > FLIP_TIME) {
            if (flipped) {
                testServo.setPosition(SERVO_LOWER_LIMIT);
                flipped = false;
            } else {
                testServo.setPosition(SERVO_UPPER_LIMIT);
                flipped = true;
            }
            lastTime = time;
        }
    }
}
