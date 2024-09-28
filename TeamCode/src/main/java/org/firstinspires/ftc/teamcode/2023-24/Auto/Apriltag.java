//package org.firstinspires.ftc.teamcode.Auto;
//
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//public class Apriltag {
//
//    private int desiredTagID = -1;
//    private int distanceFromTag = 6;
//
//    private AprilTagProcessor aprilTagProcessor;
//    private AprilTagDetection desiredTag = null;
//
//
//    private boolean blueTeam = false;
//    private boolean redTeam = false;
//
//    public AprilTagProcessor getAprilTagProcessor() {
//        return aprilTagProcessor;
//    }
//
//
//    public Apriltag(String team) {
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()) // currentgametaglibrary = centerstage + sample apriltags
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .setDrawAxes(true) // default = false
//                .setDrawCubeProjection(true) // default = false
//                .build();
//
//        if (team == "blueTeam") {blueTeam = true;} else if (team == "redTeam") {redTeam = true;}
//    }
//
//    public int setTagID(String direction) {
//        switch(direction) {
//            case "left":
//                if (blueTeam) {desiredTagID = 1;} else {desiredTagID = 4;}
//                break;
//
//            case "center":
//                if (blueTeam) {desiredTagID = 2;} else {desiredTagID = 5;}
//                break;
//
//            case "right":
//                if (blueTeam) {desiredTagID = 3;} else {desiredTagID = 6;}
//                break;
//        }
//
//        return desiredTagID;
//    }
//
//
//    public double getTagRange() {
//
//        // Step through the list of detected tags and look for a matching tag
//        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
//        for (AprilTagDetection detection : currentDetections) {
//            // Look to see if we have size info on this tag.
//            if (detection.metadata != null) {
//                //  Check to see if we want to track towards this tag.
//                if ((desiredTagID < 0) || (detection.id == desiredTagID)) {
//                    // Yes, we want to use this tag.
//                    desiredTag = detection;
//                    break;  // don't look any further.
//                }
//            }
//        }
//
//        return desiredTag.ftcPose.range;
//    }
//
//
//
//}
