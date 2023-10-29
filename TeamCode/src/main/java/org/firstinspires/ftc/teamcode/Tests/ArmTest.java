package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Robots.BradBot;

@Autonomous
@Config
public class ArmTest extends RFServoTest{

    static double FLIP_TIME = 0.5;

    //clamp: lower 0.15, upper 0.5
    //wrist: lower 0.0, upper 0.65
    //arm: lower 0.28, upper 0.98
    public static double SERVO_LOWER_LIMIT = 0.28;
    public static double SERVO_UPPER_LIMIT = 0.98;
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
