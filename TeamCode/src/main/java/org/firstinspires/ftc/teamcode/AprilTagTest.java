package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "April Tag Detection", group = "Tests")
public class AprilTagTest extends CSMethods {
    public void runOpMode() {

        setup(true);

        strafeUntilTagDetection(0);
        drive(-10);

    }
}
