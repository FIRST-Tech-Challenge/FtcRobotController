package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagTest {
    //private AprilTagProcessor tagProcessor;
    //private VisionPortal visionPortal;
    private Integer Id = 1;

    public AprilTagTest(CameraName camera){
        /*tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(false)
                .setDrawTagOutline(false)
                .build();


        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(camera)
                .setCameraResolution(new Size(640,480))
                .build();*/
    }

    public TagLocation GetPositon (AprilTagProcessor tagProcessor) {

        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == Id) {
                TagLocation blar = new TagLocation();
                blar.x = detection.ftcPose.x;
                blar.y = detection.ftcPose.y;
                blar.pitch = detection.ftcPose.pitch;
                return blar;
            }
        }
        return null;
    }

    public void setId (Integer id) {
        Id = id;
    }
    public class TagLocation {
        double x;
        double y;
        double pitch;
    }
}

