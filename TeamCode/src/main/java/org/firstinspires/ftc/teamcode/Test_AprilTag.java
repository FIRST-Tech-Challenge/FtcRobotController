package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.vision.apriltag.*;

@Autonomous(name = "April Tag Detection", group = "Tests")
//@Disabled
public class Test_AprilTag extends CSBase {
    public void runOpMode() {
        setup(true);

        AprilTagDetection a;
        while (opModeIsActive()) {
            a = tagDetections(5, 1);
            telemetry.speak(a + "\u200E");
            telemetry.addData("5", a);
            telemetry.update();
        }

    }
}
