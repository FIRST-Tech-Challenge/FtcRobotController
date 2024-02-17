package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.CSBase;

//@Autonomous(name = "April Tag Detection", group = "Tests")
@Disabled
public class Test_AprilTag extends CSBase {
    public void runOpMode() {
        setup(true);

        while (opModeIsActive()) {
            align(5);
        }

    }
}
