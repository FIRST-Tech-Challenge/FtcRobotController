package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sensors.AprilTagLogi;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagTest extends LinearOpMode {

    AprilTagLogi aprilTag = new AprilTagLogi();

    @Override
    public void runOpMode() {
        aprilTag.init(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            aprilTag.update();
            AprilTagDetection id20 = aprilTag.getTagBySpecificID(20);

            if (id20 != null) {
                telemetry.addData("id20 String", id20.toString());
            } else {
                telemetry.addData("id20", "Not visible");
            }

            telemetry.update();
        }
    }
}