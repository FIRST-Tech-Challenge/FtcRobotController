package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.teamcode.util.Util.avATPoseFtc;
import static org.firstinspires.ftc.teamcode.util.Util.avATPoseRaw;
import static org.firstinspires.ftc.teamcode.util.Util.avPose3D;
import static org.firstinspires.ftc.teamcode.util.Util.avPoint;

import android.util.Size;

import org.firstinspires.ftc.teamcode.util.VisionFilter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class Vision725 {
    public static boolean SORT_BY_X = false;

    CameraName webcam;
    public VisionPortal portal;
    public AprilTagProcessor processor;
    ArrayList<AprilTagDetection> detections;
    ArrayList<AprilTagDetection> freshDetections;
    ArrayList<AprilTagDetection> detectionsForAvg;
    private Position tagLocation = new Position(DistanceUnit.INCH,0,0,0,0);
    final int width = 640, height = 480;
    private boolean gettingAvg = false;
    private int avgPasses;
    private int targetId = -1;
    private int tagID = 0;
    private AprilTagDetection avgDetection;
    private boolean usingFreshDetections = true;
    private boolean isTagDetected = false;
    private Position cameraPosition = new Position(DistanceUnit.INCH,0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,-90, -90, 0, 0);
    private static ElapsedTime visionTimer = new ElapsedTime();
    private VisionFilter positionFilter; // Smooth AprilTag readings to reduce noise

    public Vision725(HardwareMap hm) {
        webcam = hm.get(WebcamName.class, "Webcam 1");
        processor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                //.setLensIntrinsics(821.993f, 821.993f, 330.489f, 248.997f) // For Logitech C310 Camera, Should be Default
                //.setLensIntrinsics(907.659, 907.659, 659.985, 357.874) // For Global Shutter Camera
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_25h9)
                .setTagLibrary(
                        new AprilTagLibrary.Builder()
                                .addTag(0,"beeHiveTag",1.93,DistanceUnit.INCH)
                                .addTag(1,"beeHiveTag",1.93,DistanceUnit.INCH)
                                .build()
                )
                .build();
        // For Intrinsics refer to TeamCode/src/main/res/xml/teamwebcamcalibrations.xml
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcam);
        builder.setCameraResolution(new Size(width,height));
        processor.setDecimation(1);
        builder.addProcessor(processor);
        portal = builder.build();
        visionTimer.reset();
        positionFilter = new VisionFilter(8); // Buffer size of 8 readings for moving average
    }
    public void Update() {
        if(visionTimer.milliseconds()>150){
            visionTimer.reset();
            detections = processor.getDetections();
            freshDetections = processor.getFreshDetections();
            if(freshDetections != null){
            if(freshDetections.isEmpty())
            {
                tagLocation = new Position(DistanceUnit.INCH,0, 0, 0, 0);
            }
            else
            {
            for (AprilTagDetection detection : freshDetections) {
                if (detection.metadata != null){
                    tagLocation.z = detection.ftcPose.z;
                    tagLocation.x = -detection.ftcPose.y;
                    tagLocation.y = detection.ftcPose.x;
                    tagID = detection.id;
                    positionFilter.addReading(tagLocation); // Add raw reading to filter
                }
            }
            }
            } else
            {
                tagLocation = new Position(DistanceUnit.INCH,0, 0, 0, 0);
                tagID = -1;
                positionFilter.clearBuffer(); // Clear the filter buffer when no detections are available to avoid using stale data
                // positionFilter.addReading(tagLocation); // Add zero reading when no tag detected
            }
            isTagDetected = (freshDetections != null && !freshDetections.isEmpty()); // Update detection status based on fresh detections
            // isTagDetected = tagLocation.x != 0;
        }
    }
    public void UsingFreshDetections(boolean bool) {usingFreshDetections=bool;} // Fresh Detections are as implied new detections which are used to prevent re-using the same detection data
    public boolean IsUsingFreshDetections() {return usingFreshDetections;}
    public void StartAvg(int id) {
        detectionsForAvg = new ArrayList<AprilTagDetection>();
        avgDetection = null;
        targetId = id;
        gettingAvg = true;
    }
    public void StopAvg() {
        if (detectionsForAvg == null) return;
        if (avgDetection == null && !detectionsForAvg.isEmpty()) avgDetection = detectionsForAvg.get(0);
        for (AprilTagDetection detection : detectionsForAvg) {
            avgDetection = averageDetection(avgDetection, detection);
        } //TODO: Rewrite Averaging to Average a Set of Data instead of Averaging 2 at a Time Resulting in Biased Output
        targetId = -1; gettingAvg = false;
    }
    public int DetectionsForAvgCount() {if (detectionsForAvg != null)return detectionsForAvg.size(); else return -1;}
    public void CloseVP() {portal.close();}
    public int GetTargetID() {return targetId;}
    public AprilTagDetection GetAverageDetection() {
        return avgDetection;
    }
    public ArrayList<AprilTagDetection> GetDetections() { return detections; }
    public ArrayList<AprilTagDetection> GetFreshDetections() { return freshDetections; }

    public Position GetPos(){return tagLocation;}

    /**
     * Get the filtered (smoothed) AprilTag position.
     * Returns null if buffer is not yet full (startup phase).
     * @return Smoothed Position with X, Y, Z averaged across last 8 readings
     */
    public Position GetFilteredPos() {
        return positionFilter.getFilteredReading();
    }

    /**
     * Check if filter has enough readings for reliable data.
     * @return true when buffer is full and filtered data is valid
     */
    public boolean IsFilteredDataReady() {
        return positionFilter.isBufferFull();
    }

    private AprilTagDetection averageDetection(AprilTagDetection detection1, AprilTagDetection detection2) {
        Point[] avgCorners;
        if (detection1.corners != null) {
            avgCorners = new Point[detection1.corners.length];
            for (int i = 0;i<detection1.corners.length;i++) {
                avgCorners[i] = avPoint(detection1.corners[i], detection2.corners[i]);
            }
        } else avgCorners = null;

        AprilTagDetection output = new AprilTagDetection(
                detection1.id, detection1.hamming, detection1.decisionMargin,
                avPoint(detection1.center, detection2.center),
                avgCorners,
                detection1.metadata,
                avATPoseFtc(detection1.ftcPose, detection2.ftcPose),
                avATPoseRaw(detection1.rawPose,detection2.rawPose),
                avPose3D(detection1.robotPose,detection2.robotPose),
                detection1.frameAcquisitionNanoTime
        );
        return output;
    }
    public void SortByX(ArrayList<AprilTagDetection> detections) {
        Collections.sort(detections, new Comparator<AprilTagDetection>() {
            public int compare(AprilTagDetection c1, AprilTagDetection c2) {
                if (c1.ftcPose.x > c2.ftcPose.x) return -1;
                if (c1.ftcPose.x < c2.ftcPose.x) return 1;
                return 0;
            }
        });
    }
    public boolean isTagDetected(){return isTagDetected;}
    public int getTagID(){return tagID;}
}
