package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;


//@Autonomous(name = "April Tag Detection", group = "Tests")
@Disabled
public class Test_AprilTag extends CSBase {
    public void runOpMode() {

        setup(true);

        strafe(0,dir.l);
        while (!detectTag(6));
        stopRobot();
        drive(-10);

    }
}
