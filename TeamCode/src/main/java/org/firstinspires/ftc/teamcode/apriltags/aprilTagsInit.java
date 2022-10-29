package org.firstinspires.ftc.teamcode.apriltags;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.apriltags.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class aprilTagsInit {
    double fx = 369.50;
    double fy = 369.50;
    double cx = 960;
    double cy = 540;
    double tagsize = 0.0406;//meters

    final int ID_LEFT = 1;
    final int ID_MIDDLE = 0;
    final int ID_RIGHT = 2;

    AprilTagDetection tagOfInterest = null;
    private HardwareMap hm;
    private Telemetry tm;
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetect;
    public aprilTagsInit( HardwareMap hardwareMap, Telemetry tm)
    {

        this.hm = hardwareMap;
        this.tm = tm;
    }
    public int stopAndSave()
    {
        return tagOfInterest==null ? -1 : tagOfInterest.id;//shut the program down and return the last known
    }
    public void initialize()
    {
        int cameraMonitorViewId = hm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hm.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hm.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetect = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetect);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }
    public void search()
    {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetect.getLatestDetections();
        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;
            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == ID_LEFT || tag.id == ID_MIDDLE || tag.id == ID_RIGHT) {

                    tagOfInterest = tag;
                    tm.addData("Tag of interest is in sight!\n\nID:", tag.id+1);
                    tagFound = true;
                    break;
                }
            }
            if(!tagFound)
            {
                tm.addLine("Tag of interest not in sight, but there has been a tag spotted!");

                if(tagOfInterest == null)
                {
                    tm.addLine("(The tags you are looking for have never been seen)");
                }
                else
                {
                    tm.addLine("\nLast tag seen at: " + tagOfInterest.id);
                }
            }

        }
        else
        {
            tm.addLine("Don't see any tags");
            if(tagOfInterest != null)
            {
                tm.addLine("Last tag seen: " + tagOfInterest.id);
            }
        }
        tm.update();

    }


}
