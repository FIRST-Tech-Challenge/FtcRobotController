package org.firstinspires.ftc.teamcode.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * William
 * same as RFServoTest but for RFDualServo
 */
@Disabled

public abstract class RFDualServoTest extends LinearOpMode{
    RFServo testServo;
    RFServo testServo2;
    double lastTime = 0;
    boolean flipped = false;
    double FLIP_TIME;

    double SERVO_LOWER_LIMIT;
    double SERVO_UPPER_LIMIT;

    /**
     * Constructs RFDualServoTest class
     * @param p_testServo RFServo that needs to be tested
     * @param p_testServo2 second RFServo that needs to be tested
     * @param p_FLIP_TIME time it takes for servo to flip from lower limit to upper limit
     * @param p_SERVO_LOWER_LIMIT lower limit of servo
     * @param p_SERVO_UPPER_LIMIT upper limit of servo
     */
    public RFDualServoTest(RFServo p_testServo, RFServo p_testServo2, double p_FLIP_TIME, double p_SERVO_LOWER_LIMIT, double p_SERVO_UPPER_LIMIT) {
        testServo2 = p_testServo2;
        testServo = p_testServo;
        FLIP_TIME = p_FLIP_TIME;
        SERVO_LOWER_LIMIT = p_SERVO_LOWER_LIMIT;
        SERVO_UPPER_LIMIT = p_SERVO_UPPER_LIMIT;
    }

    /**
     * Continuously sets servos to lower limit and upper limit.
     * Logs that the servos have been flipped to max positions.
     * Logs to RFDualServo & general logs.
     * Logs to second finest level.
     * Does not update a state machine.
     */
    public void autoLoop() {
        if (time - lastTime > FLIP_TIME) {
            if (flipped) {
                testServo.setPosition(SERVO_LOWER_LIMIT);
                testServo2.setPosition(1 - SERVO_LOWER_LIMIT);
                flipped = false;
            } else {
                testServo.setPosition(SERVO_UPPER_LIMIT);
                testServo2.setPosition(1 - SERVO_UPPER_LIMIT);
                flipped = true;
            }
            lastTime = time;
        }
    }
}
