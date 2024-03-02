package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Base;

//@Autonomous(name = "Auto Lift", group = "Tests")
@Disabled
public class Test_AutoLift extends Base {
    public void runOpMode() {
        setup();
        moveLift(GOAL_ENCODERS);
        dropPixels();
        retractLift();
    }
}
