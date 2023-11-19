package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Config
@Disabled

@Autonomous(name = "wristTest")
public class wristTest extends RFServoTest{

    static double FLIP_TIME = 0.5;

    //clamp: lower 0.15, upper 0.5
    //wrist: lower 0.0, upper 0.65
    //arm: lower 0.28, upper 0.98
    public static double FLAT = 1.0, HOLD = 0.7, FLIP = 0.17, DROP = 0.0;
    public static int target = 0;
    private int atTarg = 0;

    /**
     * Calls autoLoop() function from RFServoTest (see RFServoTest class).
     */
    public void runOpMode() {
        initialize("wristServo", FLIP_TIME, SERVO_LOWER_LIMIT, SERVO_UPPER_LIMIT);
        waitForStart();
        double[] pussitions = {FLAT, HOLD, FLIP, DROP};
        while (opModeIsActive()) {
            if (target != atTarg) {
                flipTo(pussitions[target]);
                atTarg = target;
            }
            packet.put("atTarg", atTarg);
            robot.update();
        }
    }
}
