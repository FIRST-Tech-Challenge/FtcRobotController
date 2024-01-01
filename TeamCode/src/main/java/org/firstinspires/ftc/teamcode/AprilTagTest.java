package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


//@Autonomous(name = "April Tag Detection", group = "Tests")
@Disabled
public class AprilTagTest extends CSMethods {
    public void runOpMode() {

        setup(true);

        strafeUntilTagDetection(0);
        drive(-10);

    }
}
