package org.firstinspires.ftc.teamcode.Tests;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class ArmTest extends RFServoTest{

    static double FLIP_TIME = 1;
    static double SERVO_LOWER_LIMIT = 0;
    static double SERVO_UPPER_LIMIT = 1;

    /**
     * Constructs super from RFServoTest
     */
    public ArmTest() {
        super(new RFServo("armServo", 1.0), FLIP_TIME, SERVO_LOWER_LIMIT, SERVO_UPPER_LIMIT);
    }

    /**
     * Calls autoLoop() function from RFServoTest (see RFServoTest class).
     */
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            super.autoLoop();
        }
    }
}
