package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "April Tag Detection", group = "Tests")
//@Disabled
public class Test_AprilTag extends CSBase {
    public void runOpMode() {
        setup(true);

        while (opModeIsActive()) {
            align(5);
        }

    }
}
