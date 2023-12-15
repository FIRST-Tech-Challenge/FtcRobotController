package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagTest {
    AprilTagProcessor TagProcessor  = new AprilTagProcessor.Builder()
        .setDrawAxes(true)
            .setDrawCubeProjection(true)
    .build();

}
