package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Robots.BradBot;

@Autonomous
@Config
public class PreloadTest extends RFServoTest{

    static double FLIP_TIME = 0.5;
    public static double SERVO_LOWER_LIMIT = 0.15;
    public static double SERVO_UPPER_LIMIT = 0.5;
    /**
     * Calls autoLoop() function from RFServoTest (see RFServoTest class).
     */
    public void runOpMode() {
        initialize("preloadServo", FLIP_TIME, SERVO_LOWER_LIMIT, SERVO_UPPER_LIMIT);
        waitForStart();
        while (opModeIsActive()) {
            autoLoop();
            sleep(10000);
        }
    }
}
