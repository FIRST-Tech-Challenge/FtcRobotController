package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Robots.BradBot;

@Autonomous
public class ArmTest extends RFServoTest{

    static double FLIP_TIME = 0.5;

    //clamp: lower 0.2, upper 0.5
    //wrist: lower 0.0, mid (to get over slides) 0.625, upper 0.25
    //arm: lower 0.2, upper 0.7
    static double SERVO_LOWER_LIMIT = 0.8;
    static double SERVO_UPPER_LIMIT = 0.8;
    /**
     * Calls autoLoop() function from RFServoTest (see RFServoTest class).
     */
    public void runOpMode() {
        initialize("armServo", FLIP_TIME, SERVO_LOWER_LIMIT, SERVO_UPPER_LIMIT);
        waitForStart();
        while (opModeIsActive()) {
            autoLoop();
        }
    }
}
