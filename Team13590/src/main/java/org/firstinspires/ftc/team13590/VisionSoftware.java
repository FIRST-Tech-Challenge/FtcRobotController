package org.firstinspires.ftc.team13590;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.SortOrder;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

public class VisionSoftware extends RobotHardware{

    public VisionSoftware(LinearOpMode opmode) {super(opmode);}


    public static class colorDetector extends VisionSoftware{
        public colorDetector(LinearOpMode opmode) {super(opmode);}

        // init vision variables
        public ColorBlobLocatorProcessor primaryColorProcessor;
        public ColorBlobLocatorProcessor secondaryColorProcessor;

        public List<ColorBlobLocatorProcessor.Blob> primaryBlobList;
        public List<ColorBlobLocatorProcessor.Blob> secondaryBlobList;

        public VisionPortal portalColor;

        /**
         *
         * @param leftUp Top Left Point of the ROI you wish to set
         * @param rightDown Bottom Right Point of the ROI you wish to set
         *                  These Points can only make shapes with all perpendicular angles (only squares/rectangles)
         * @param blobList Pass the list (blob list) you wish to filter
         */
        public void filterBySetROI(Point leftUp, Point rightDown, List<ColorBlobLocatorProcessor.Blob> blobList) {
            ArrayList<ColorBlobLocatorProcessor.Blob> toRemove = new ArrayList<>();

            Point leftDown = new Point(leftUp.x, rightDown.y);
            Point rightUp = new Point(rightDown.x, leftUp.y);

            Point[] pointsSet = {leftUp, leftDown, rightUp, rightDown};

            Imgproc.minAreaRect( new MatOfPoint2f(pointsSet));

            for(ColorBlobLocatorProcessor.Blob b : blobList)
            {
                double bCenterX = b.getBoxFit().center.x;
                double bCenterY = b.getBoxFit().center.y;

                if (bCenterX <= leftUp.x || bCenterY >= leftUp.y ||
                        bCenterX >= rightDown.x || bCenterY <= rightDown.y)
                {
                    toRemove.add(b);
                }
            }

            blobList.removeAll(toRemove);
        }

        /**
         *
         * @param color What color you want (IN STRING VALUE), BLUE, RED, or YELLOW
         * @param doublePortal If you want to reset the {@link VisionPortal} or not, true is yes, false is no
         * @param left How far left from the center the border should be, range of 1,-1
         * @param top How far up from the center the border should be, range of 1,-1
         * @param right How far right from the center the border should be, range of 1,-1
         * @param bottom How far down from the center the border should be, range of 1,-1
         */
        public void visionInit (String color, boolean doublePortal,
                                double left, double top, double right, double bottom) {

            switch (color) { // CUTTING EDGE CODE!!!!
                case "BLUE":
                    primaryColorProcessor = new ColorBlobLocatorProcessor.Builder()
                            .setBoxFitColor(Color.rgb(255,0,255))
                            .setContourColor(Color.rgb(0,255,100))
                            .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                            .setRoi(ImageRegion.asUnityCenterCoordinates(left, top, right, bottom))  // search central 1/4 of camera view
                            .setDrawContours(true)                        // Show contours on the Stream Preview
                            .setBlurSize(5)                               // Smooth the transitions between different colors in image
                            .build();
                    break;
                case "RED":
                    primaryColorProcessor = new ColorBlobLocatorProcessor.Builder()
                            .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                            .setRoi(ImageRegion.asUnityCenterCoordinates(left, top, right, bottom))  // search central 1/4 of camera view
                            .setDrawContours(true)                        // Show contours on the Stream Preview
                            .setBlurSize(5)                               // Smooth the transitions between different colors in image
                            .build();
                    break;
            }
            secondaryColorProcessor = new ColorBlobLocatorProcessor.Builder()
                    .setRoiColor(Color.rgb(0,255,0))
                    .setBoxFitColor(Color.rgb(255,255,60))
                    .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                    .setRoi(ImageRegion.asUnityCenterCoordinates(left, top, right, bottom))  // search central 1/4 of camera view
                    .setDrawContours(true)                        // Show contours on the Stream Preview
                    .setBlurSize(5)                               // Smooth the transitions between different colors in image
                    .build();


            if (doublePortal) {
                 portalColor = new VisionPortal.Builder()
                        .addProcessors(primaryColorProcessor, secondaryColorProcessor)
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .setCameraResolution(new Size(1920, 1080))
                        .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .build();
                portalColor.setProcessorEnabled(secondaryColorProcessor, false);
            } else {
                portalColor = new VisionPortal.Builder()
                    .addProcessor(primaryColorProcessor)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .setCameraResolution(new Size(1920, 1080))
                    .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .build();
            }
            portalColor.setProcessorEnabled(primaryColorProcessor, false);

            myOpMode.telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
            myOpMode.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        }

        @SuppressLint("DefaultLocale")
        public void activeDetector(Point topLeft, Point bottomRight, String processors) {

            if (Objects.equals(processors, "PRIMARY")) {
                myOpMode.telemetry.addData("SCANNING", "...\n" +
                        " \nPRIMARY CAMERA PORTAL ACTIVE\n");

                primaryBlobList = primaryColorProcessor.getBlobs(); // set list to whatever the camera found

                ColorBlobLocatorProcessor.Util.filterByArea(1000, 40000, primaryBlobList);  // filter out very small blobs.
                ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, primaryBlobList); // have the largest blobs come first in the list
                myOpMode.telemetry.addLine(" Area Density Aspect  Center");

                // Display the size (area) and center location for each Blob.
                for (ColorBlobLocatorProcessor.Blob b : primaryBlobList) // telemetry the blobs found
                {
                    RotatedRect boxFit = b.getBoxFit();
                    myOpMode.telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)  %3.1f",
                            b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y, boxFit.angle));
                    myOpMode.telemetry.addData(String.valueOf(primaryBlobList.indexOf(b)), "");
                }
                myOpMode.telemetry.update();
            } else if (Objects.equals(processors, "BOTH")) {
                    myOpMode.telemetry.addData("SCANNING", "...\n" +
                            " \nPRIMARY CAMERA PORTAL ACTIVE\n");

                    primaryBlobList = primaryColorProcessor.getBlobs(); // set list to whatever the camera found
                    secondaryBlobList = secondaryColorProcessor.getBlobs();

                    filterBySetROI(topLeft, bottomRight, primaryBlobList);
                    filterBySetROI(topLeft, bottomRight, secondaryBlobList);

                    ColorBlobLocatorProcessor.Util.filterByArea(1000, 40000, primaryBlobList);  // filter out very small blobs.
                    ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, primaryBlobList); // have the largest blobs come first in the list
                    ColorBlobLocatorProcessor.Util.filterByArea(1000, 40000, secondaryBlobList);
                    ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, secondaryBlobList); // have the largest blobs come first in the list

                    myOpMode.telemetry.addLine(" Primary Area, Density, Aspect,  Center, Angle");

                    // Display the size (area) and center location for each Blob.
                    for(ColorBlobLocatorProcessor.Blob b : primaryBlobList) // telemetry the blobs found
                    {
                        RotatedRect boxFit = b.getBoxFit();
                        myOpMode.telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d) %3.1f",
                                b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y, boxFit.angle));
                        myOpMode.telemetry.addData("Angle", String.valueOf(boxFit.angle));
                    }

                    myOpMode.telemetry.addLine(" Secondary Area, Density, Aspect,  Center, Angle");

                    // Display the size (area) and center location for each Blob.
                    for(ColorBlobLocatorProcessor.Blob b : secondaryBlobList) // telemetry the blobs found
                    {
                        RotatedRect boxFit = b.getBoxFit();
                        myOpMode.telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d) %3.1f",
                                b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y, boxFit.angle));
                        myOpMode.telemetry.addData("Angle", String.valueOf(boxFit.angle));
                    }
                    myOpMode.telemetry.update();
            }
        }
    }

    public static class aptDetector extends VisionSoftware{
        public aptDetector(LinearOpMode opmode) {super(opmode);}

        public AprilTagProcessor APTprocessor;
        public VisionPortal portalAPT;

        public boolean targetFound = false;
        public AprilTagDetection detectedTag = null;

        public Position cameraPosition = new Position(DistanceUnit.INCH,
                0,-6,0,0);
        public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -270, 0,0);
        public void visionInit(){
            /* set up processor; only one camera will use the APT detector, also there is no need to detect different
                types of APT like in the colorDetector. (what im saying is there is no need for the switch cases like before)
             */
            APTprocessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setLensIntrinsics(1130.80338323, 1130.80338323, 1280.21111078, 368.731101737)
                    .setCameraPose(cameraPosition, cameraOrientation)
                    .build();

            portalAPT = new VisionPortal.Builder()
                    .addProcessor(APTprocessor)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .setCameraResolution(new Size(1920, 1080))
                    .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .build();



            portalAPT.setProcessorEnabled(APTprocessor, false);
        }

        @SuppressLint("DefaultLocale")
        public void activeAPTscanner(int DESIRED_TAG_ID) {
            List<AprilTagDetection> currentDetections = APTprocessor.getDetections();
            if (!currentDetections.isEmpty()) { // Check if you see an APT
                if (DESIRED_TAG_ID == -2){
                    // check if you recognize all tags
                    for (AprilTagDetection detection : currentDetections){
                        if (detection.metadata == null){
                            myOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary; removing", detection.id);
                            currentDetections.remove(detection);
                        }
                    }
                    // check that we still have something in the list (other check doesn't work cuz we hadn't filtered yet)
                    if (!currentDetections.isEmpty()) {
                        // start with first index (we have a small list so it's ok for this type of sort)
                        AprilTagDetection lowest = currentDetections.get(0);
                        // go thru every value and save the one closest to you
                        for (int i = 1; i < currentDetections.size(); i++) {
                            if (currentDetections.get(i).ftcPose.range < lowest.ftcPose.range) {
                                lowest = currentDetections.get(i);
                            }
                        }
                        //
                        targetFound = true;
                        detectedTag = lowest;
                    } else {targetFound = false;}
                } else {
                    for (AprilTagDetection detection : currentDetections) {
                        // Look to see if we have size info on this tag.
                        if (detection.metadata != null) {
                            //  Check to see if we want to track towards this tag.
                            if ((DESIRED_TAG_ID == -1) || (detection.id == DESIRED_TAG_ID)) {
                                // Yes, we want to use this tag.
                                targetFound = true;
                                detectedTag = detection;
                                break;  // don't look any further.
                            } else {
                                // This tag is in the library, but we do not want to track it right now.
                                myOpMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        } else {
                            // This tag is NOT in the library, so we don't have enough information to track to it.
                            myOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
                    }
                }
            } else { targetFound = false; }// Communicate that there is no APT in view; the previously detected APT will stay in detectedTag

            // Telemetry what is found
            if (targetFound) {
                myOpMode.telemetry.addData("\n>","Target Found!!!\n");
                myOpMode.telemetry.addData("Found", "ID %d (%s)", detectedTag.id, detectedTag.metadata.name);
                myOpMode.telemetry.addData("Range",  "%5.1f inches", detectedTag.ftcPose.range);
                myOpMode.telemetry.addData("Bearing","%3.0f degrees", detectedTag.ftcPose.bearing);
                myOpMode.telemetry.addData("Yaw","%3.0f degrees", detectedTag.ftcPose.yaw);
                // Display the coordinate position for the APT detected
                myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (APT inch)", detectedTag.ftcPose.x, detectedTag.ftcPose.y, detectedTag.ftcPose.z));
            } else {
                myOpMode.telemetry.addData("\n>","No Target Found...\n");
            }
            // the following telemetry messages are for troubleshooting...
            // Display the PRY (pitch, roll, yaw) for the robot
            if (detectedTag != null) {
                myOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detectedTag.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detectedTag.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detectedTag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                // Display the coordinate position for the robot
                myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detectedTag.robotPose.getPosition().x,
                        detectedTag.robotPose.getPosition().y,
                        detectedTag.robotPose.getPosition().z));
                cameraPosition = detectedTag.robotPose.getPosition();
                cameraOrientation = detectedTag.robotPose.getOrientation();
            }
        }

    }
}
