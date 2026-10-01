/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.aprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
public class AprilTagWebcam extends OpMode {
    aprilTagWebcam aprilTagWebcam = new aprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();

        AprilTagDetection id20 = aprilTagWebcam.getTagByspecificId(20);
        if (id20 != null) {
            aprilTagWebcam.displayDetectionTelemetry(id20);
        } else {
            telemetry.addLine("Tag 20 not detected");
        }

        AprilTagDetection id24 = aprilTagWebcam.getTagByspecificId(24);
        if (id24 != null) {
            aprilTagWebcam.displayDetectionTelemetry(id24);
        } else {
            telemetry.addLine("Tag 24 not detected");
        }
    }

}
*/