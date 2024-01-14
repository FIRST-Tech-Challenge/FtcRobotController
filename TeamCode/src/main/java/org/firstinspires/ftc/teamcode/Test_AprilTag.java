package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;


//@Autonomous(name = "April Tag Detection", group = "Tests")
@Disabled
public class Test_AprilTag extends CSBase {
    public void runOpMode() {

        setup(true);

        strafeUntilTagDetection(dir.left,0);
        drive(-10);

    }
}
