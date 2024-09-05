package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystems.Apriltag;
import org.firstinspires.ftc.teamcode.common.subsystems.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(name = "Vision2")
public class VisionPortalOp2 extends CommandOpMode {
    VisionPortal visionPortal = null;
    Apriltag aprilTag = null;
    Telemetry tel = null;
    @Override
    public void initialize() {
        aprilTag = new Apriltag();
        visionPortal = new VisionPortal("camera1", Arrays.asList(aprilTag.getProcessor()));
        tel = FtcDashboard.getInstance().getTelemetry();
    }


    @Override
    public void run() {
        super.run();
        List<AprilTagDetection> tags = aprilTag.getTags();

        for (AprilTagDetection tag:  tags) {
//            String poseData = "${tag.ftcPose.x}, ${tag.ftcPose.y}, ${tag.ftcPose.z}";
            String poseData = String.format("%s %s %s", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z);
            tel.addData(String.valueOf(tag.id), poseData);
        }
        tel.update();
    }
}
