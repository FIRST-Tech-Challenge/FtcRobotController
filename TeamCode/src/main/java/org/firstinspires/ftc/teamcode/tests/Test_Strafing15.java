package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CSBase;

@Disabled
public class Test_Strafing15 extends CSBase {
    public void runOpMode() {
        setup(true);
        strafe(15, dir.left);
    }
}
