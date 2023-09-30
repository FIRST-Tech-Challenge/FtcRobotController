package org.firstinspires.ftc.teamcode.robots.csbot.vision.provider;

import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.wrapAngle;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.Position;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.Target;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.pipeline.DPRGCanDetectorPipeline;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.Map;

/**
 * @author Iron Reign Coding Team
 */

@Config  (value = "AA_PP_6CAN_VISION_PROVIDER")
public class DPRGCanDetectorProvider extends VisionProvider {
    private Bitmap noCameraBitmap;
    private OpenCvCamera camera;
    private DPRGCanDetectorPipeline pipeline;
    private volatile boolean cameraOpened;
    private List<Target> frameDetections = new ArrayList<>();
    public List<Target> getFrameDetections() {
        return frameDetections;
    }
    private List<Target> uniqueCans = new ArrayList<>();
    public List<Target> getUniqueCans() {
        return uniqueCans;
    }
    private long lastFrameTimestamp;

    // Constants
    private static final String TELEMETRY_NAME = "DPRG 6Can Vision Provider";
    public static int WEBCAM_WIDTH = 320;
    public static int WEBCAM_HEIGHT = 180;
    public static double distanceThreshold = 6; //this is in inches - detections closer than this threshold are considered duplicates

    @Override
    public void initializeVision(HardwareMap hardwareMap) {
        pipeline = new DPRGCanDetectorPipeline();
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        noCameraBitmap = Bitmap.createBitmap(320, 240, Bitmap.Config.RGB_565);
        Mat noCameraMat = new Mat(240, 320, CvType.CV_8UC3);
        Imgproc.putText(noCameraMat, "Webcam Could", new Point(40, 110), Imgproc.FONT_HERSHEY_SIMPLEX,
                1, new Scalar(255, 0, 0), 3);
        Imgproc.putText(noCameraMat, "Not Be Opened", new Point(40, 150), Imgproc.FONT_HERSHEY_SIMPLEX,
                1, new Scalar(255, 0, 0), 3);
        Utils.matToBitmap(noCameraMat, noCameraBitmap);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
                cameraOpened = true;
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void shutdownVision() {
//        if(cameraOpened) {
            try {
                camera.stopStreaming();
                camera.closeCameraDevice();
            } catch(Exception e) {

            }
//        }
//        cameraOpened = false;
    }

    @Override
    public Position getPosition() {
        return cameraOpened ? pipeline.getLastPosition() : Position.HOLD;
    }

    @Override
    public void reset() {

    }

    @Override
    public boolean canSendDashboardImage() {
        return true;
    }

    @Override
    public Bitmap getDashboardImage() {
        return cameraOpened ? pipeline.getDashboardImage() : noCameraBitmap;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = super.getTelemetry(debug);

        if(debug && cameraOpened) {

            telemetryMap.put("Frame Count", camera.getFrameCount());
            telemetryMap.put("FPS", Misc.formatInvariant("%.2f", camera.getFps()));
            telemetryMap.put("Total frame time ms", camera.getTotalFrameTimeMs());
            telemetryMap.put("Pipeline time ms", camera.getPipelineTimeMs());
            telemetryMap.put("Overhead time ms", camera.getOverheadTimeMs());
            telemetryMap.put("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
            telemetryMap.put("Largest Coordinate", Arrays.toString(pipeline.getLargestCoordinate()));
            telemetryMap.put("Area in pixels", pipeline.getLargestAreaPixels());

            for (Target can: frameDetections){
                telemetryMap.put(Integer.toString(can.getTargetNumber())+"Timestamp", can.getTimeStamp());
                //telemetryMap.put("ID", can.getTargetNumber());
                telemetryMap.put(Integer.toString(can.getTargetNumber())+"Orientation Angle", can.getOrientation());
                telemetryMap.put(Integer.toString(can.getTargetNumber())+"Width", can.getWidthPixels());
                telemetryMap.put(Integer.toString(can.getTargetNumber())+"Height", can.getHeightPixels());
                telemetryMap.put(Integer.toString(can.getTargetNumber())+"Aspect Ratio", can.getAspectRatio());
                telemetryMap.put(Integer.toString(can.getTargetNumber())+"Camera Heading", can.getCameraHeading());
                telemetryMap.put(Integer.toString(can.getTargetNumber())+"Camera Distance", can.getCameraDistance());
                telemetryMap.put(Integer.toString(can.getTargetNumber())+"X", can.getFieldPosition().getX());
                telemetryMap.put(Integer.toString(can.getTargetNumber())+"Y", can.getFieldPosition().getY());
            }
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    @Override
    public void updateVision() {
        long timestamp = 0;

        //get fresh frame detections
        frameDetections.clear();
        try {
            frameDetections = pipeline.getDetectedCans();
            if (frameDetections.size()>0) timestamp = frameDetections.get(0).getTimeStamp();
        }
        catch (ConcurrentModificationException ex) {
            //seems to happen occasionally when the detectedCans object is being populated in the pipeline while we are trying to retrieve it
            //do nothing - rely on most recently collected detections
        }
        //test to see if this is a fresh set - they will have a greater timestamp

        if (timestamp>lastFrameTimestamp){
            //process the frame detections
            //estimate the locations of the cans
            for (Target newCan : frameDetections) {
                boolean isDuplicate = false;

                //estimate the distance to the can based on the height parameter since that scales
                //inversely but fairly linearly with the distance from the camera - at least for upright
                //cans. Right now we are assuming that cans are upright at the beginning of play
                //experimentally, these values were obtained for a can at various measured distances
                //4ft = 27 pixels high
                //1ft = 81 pixels high
                //8" = 105 pixels - closest reliable
                double cameraDistance = estimateSodacanDistance(newCan.getHeightPixels()); //distance from camera based on measurements
                //will need to add the distance to the center of the robot
                double canDistance = cameraDistance + robot.driveTrain.getChassisLength(); //from robot chassis' center of rotation
                canDistance += 4.25; //to account for the camera sitting forward of the distance sensor target

                //estimate the angle to the can
                //experimentally angles were determined by laying a large printed protractor on the
                //ground and overpainting the decade marks on a captured frame:
                //https://cdn.discordapp.com/attachments/531247253222981649/1104777072917872690/image.png
                //the sample can be found in the team's #code discord channel on May 7th, 2023
                //message link:
                //https://discord.com/channels/531243513615220748/531247253222981649/1104777073135988888
                //at the top of the frame a range of 60 degrees corresponds to a span of 295 centered on 160
                //at the bottom 60 degrees is a span of 237 also centered on 160
                //the height of the frame is 180
                double x = newCan.getCentroid().getX();
                double y = newCan.getCentroid().getY();
                double heightFactor = (180.0-y)/180.0;
                double deltaThirty = (58.0 * heightFactor + 237.0)/2; //pixel width at a given height equivalent to 30 degrees from center
                double frameDegrees =  wrapAngle(-(x-160.0) / deltaThirty * 30);
                newCan.setCameraHeading(frameDegrees);
                newCan.setCameraDistance(cameraDistance);
                double targetHeadingRad = robot.driveTrain.poseEstimate.getHeading() +
                        Math.toRadians(robot.underarm.getTurretTargetAngle()) + Math.toRadians(frameDegrees);

                //general trig for the can coordinates given a distance and heading
                //can_x = robot_x + distance * cos(heading)
                //can_y = robot_y + distance * sin(heading)
                double can_x = (robot.driveTrain.poseEstimate.getX()+canDistance*Math.cos(targetHeadingRad));
                double can_y = robot.driveTrain.poseEstimate.getY()+canDistance*Math.sin(targetHeadingRad);
                newCan.setFieldPosition(new Vector2d(can_x,can_y));

                for (Target existingCan : uniqueCans) {
                    double distance = newCan.getFieldPosition().distTo(existingCan.getFieldPosition());

                    if (distance < distanceThreshold) {
                        isDuplicate = true;
                        //replace the target with the newer version
                        uniqueCans.set(uniqueCans.indexOf(existingCan), newCan);
                        break;
                    }
                }

                if (!isDuplicate) {
                    uniqueCans.add(newCan);
                }

            }
            //send the targets for visualization
            //robot.setTargets(uniqueCans);
            robot.setTargets(frameDetections);

            //cache the timestamp
            lastFrameTimestamp = timestamp;
        }
    }

    //this estimator is based on fitting a power series trendline to the sample data in a spreadsheet as suggested by GPT-4
    //https://docs.google.com/spreadsheets/d/1-Ge_xL1NMW1dekoahnYKda2tjjA1VF73Qep7wM35otg/edit?usp=sharing
    public static double estimateSodacanDistance(double pixelHeight) {
        double a = 2274;
        double b = -1.18;
        return a * Math.pow(pixelHeight, b);
    }

    public Target getStartingCan(){
        //find the nearest can that is close to the centerline
        // can where abs of the x coordinate < 12
        // and y is smallest (closest to start)
        // if that doesn't find a target, get keep increasing the zone until a can is found
        double smallestY = 10*12;
        Target returnableCan = null;

        List<Target> shortlist = new ArrayList<>();

        for (Target can: uniqueCans){
            if (Math.abs(can.getFieldPosition().getX())<12){
                shortlist.add(can);
            }
        }
        for (Target can: shortlist){
            if (can.getFieldPosition().getY() < smallestY)
            {
                smallestY = can.getFieldPosition().getY();
                returnableCan = can; //this is the smallest one so far
            }
        }
        return returnableCan; //could be null
    }

    public Target GetNearest(List<Target> targets, Vector2d location){
        Target closest = null;
        double smallest = Double.MAX_VALUE;
        for (Target target: targets){
            double dist = location.distTo(target.getFieldPosition());
            if (dist<smallest) {
                smallest = dist;
                closest = target;
            }

        }
        return closest; //could be a null Target
    }

    public Target GetNearest(List<Target> targets, Target target){
        return GetNearest(targets, target.getFieldPosition());
    }

    public Target GetNearest(List<Target> targets, Pose2d pose){
        return GetNearest(targets, new Vector2d(pose.getX(), pose.getY()));
    }
}

