package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.susbsystems.Vision725;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;

import java.util.ArrayList;

@TeleOp (name = "Vision Test", group = "SubsysTest")
public class visionTest extends OpMode {
    Vision725 vision;
    ButtonBlock toggleAvg, toggleUFD;
    Position tagLocation = new Position(DistanceUnit.INCH,0,0,0,0);
    Position detTagPose = new Position(DistanceUnit.INCH,0,0,0,0);

    boolean averaging = false;
    @Override
    public void init() {
        vision = new Vision725(hardwareMap);
        toggleAvg = new ButtonBlock() .onTrue(() -> {toggleAvg();});
        toggleUFD = new ButtonBlock() .onTrue(() -> {toggleUFD();});
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        toggleAvg.update(gamepad1.a);
        toggleUFD.update(gamepad1.b);
        vision.Update();
        ArrayList<AprilTagDetection> detections = vision.GetDetections();
        ArrayList<AprilTagDetection> freshDetections = vision.GetFreshDetections();

        telemetry.addLine("===== APRILTAG VERIFICATION =====");
        telemetry.addData("Detections Size", detections == null ? -1 : detections.size());
        telemetry.addData("Fresh Detections Size", freshDetections == null ? -1 : freshDetections.size());
        telemetry.addData("Vision725 isTagDetected", vision.isTagDetected());
        telemetry.addLine("If detections > 0 but fresh = 0, the tag is visible but no new frame was returned this loop.");

        if (detections != null)
            for (AprilTagDetection detection : detections) {
                if (detection == null || detection.robotPose == null || detection.robotPose.getPosition() == null) continue;
                telemetry.addLine("----- Detection Detail -----");
                telemetry.addData("ID", detection.id);
                telemetry.addData("Metadata Present", detection.metadata != null ? "YES" : "NO");
                Position detPose = detection.robotPose.getPosition();
                telemetry.addLine(String.format("Robot id: %03d x: %.2f y: %.2f z: %.2f", detection.id, detPose.x, detPose.y, detPose.z) );
                if (detection.ftcPose != null) {
                    AprilTagPoseFtc tagdetPose = detection.ftcPose;
                    telemetry.addLine(String.format("FTC id: %03d x: %.2f y: %.2f z: %.2f", detection.id, tagdetPose.x, tagdetPose.y, tagdetPose.z) );
                    tagLocation.z = detection.ftcPose.z;
                    tagLocation.x = -detection.ftcPose.y;
                    tagLocation.y = detection.ftcPose.x;
                    telemetry.addLine(String.format("FTC-Corrected id: %03d x: %.2f y: %.2f z: %.2f", detection.id, tagLocation.x, tagLocation.y, tagLocation.z) );
                } else {
                    telemetry.addLine("FTC Pose: NULL");
                }
                if (detection.rawPose != null) {
                    AprilTagPoseRaw rawdetpose = detection.rawPose;
                    telemetry.addLine(String.format("Raw id: %03d x: %.2f y: %.2f z: %.2f", detection.id, rawdetpose.x, rawdetpose.y, rawdetpose.z) );
                } else {
                    telemetry.addLine("Raw Pose: NULL");
                }
            }
       /* ArrayList<AprilTagDetection> freshDetections = vision.GetFreshDetections();
        if (freshDetections != null)
            for (AprilTagDetection detection : freshDetections) {
                if (detection == null || detection.robotPose == null || detection.robotPose.getPosition() == null) continue;
                Position detPose = detection.robotPose.getPosition();
                telemetry.addLine(String.format("Fresh id: %03d x: %.2f y: %.2f z: %.2f", detection.id, detPose.x, detPose.y, detPose.z) );
            }*/

        telemetry.addLine("Gamepad1 A: Toggle Avg B: Toggle Fresh");
        telemetry.addData("Target Tag Id", vision.GetTargetID());
        telemetry.addData("Averaging Data Set Size",vision.DetectionsForAvgCount());
        telemetry.addData("Averaging",averaging);
        telemetry.addData("UsingFreshDetections",vision.IsUsingFreshDetections());
        /*if (vision.GetAverageDetection() != null) {
            Position avgPose = vision.GetAverageDetection().robotPose.getPosition();
            telemetry.addData("Average Tag", String.format("x: %.2f y: %.2f z: %.2f", avgPose.x, avgPose.y, avgPose.z) );
        }
        detTagPose = vision.GetPos();
        if(vision.GetPos() != null){
            telemetry.addLine(String.format("[Corrected]x: %.2f y: %.2f z: %.2f", detTagPose.x, detTagPose.y, detTagPose.z) );
        }
        else {
            telemetry.addLine("Null Tag Position");
        }*/
        detTagPose = vision.GetPos();
        telemetry.addLine("===== VISION725 STORED POSITION =====");
        telemetry.addLine(String.format("[Loop Computed] x: %.2f y: %.2f z: %.2f", tagLocation.x, tagLocation.y, tagLocation.z) );
        telemetry.addLine(String.format("[Vision725 GetPos] Detected %b x: %.2f y: %.2f z: %.2f", vision.isTagDetected(),detTagPose.x, detTagPose.y, detTagPose.z) );
        telemetry.addLine("If Loop Computed is non-zero but GetPos is zero, Vision725.Update() is resetting stored position.");
        telemetry.update();
    }
    private void toggleAvg() {
        if (averaging) {
            vision.StopAvg();
        } else {
            vision.StartAvg(vision.GetDetections().get(0).id);
        }
        averaging = !averaging;
    }
    private void toggleUFD() {
        if (vision.IsUsingFreshDetections())
            vision.UsingFreshDetections(false);
        else
            vision.UsingFreshDetections(true);
    }
}
