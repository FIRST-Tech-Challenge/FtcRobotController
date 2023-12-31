package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "April Tag Detection", group = "Tests")
public class AprilTagTest extends CSMethods {
    public void runOpMode() {
        waitForStart();

        strafeWithTagDetection(0);
        drive(-10);

    }
}
