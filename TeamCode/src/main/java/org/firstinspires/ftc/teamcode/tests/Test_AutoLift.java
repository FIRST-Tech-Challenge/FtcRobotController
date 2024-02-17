package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CSBase;

//@Autonomous(name = "Auto Lift", group = "Tests")
@Disabled
public class Test_AutoLift extends CSBase {
    public void runOpMode() {
        setup();
        moveLift(GOAL_ENCODERS);
        dropPixels();
        retractLift();
    }
}
