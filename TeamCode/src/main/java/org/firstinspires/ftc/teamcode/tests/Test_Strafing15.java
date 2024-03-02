package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Base;

@Disabled
public class Test_Strafing15 extends Base {
    public void runOpMode() {
        setup(true);
        strafe(15, dir.left);
    }
}
