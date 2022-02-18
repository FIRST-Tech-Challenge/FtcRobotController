//package org.firstinspires.ftc.teamcode.Visual;
//
//import android.graphics.Bitmap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
//import org.firstinspires.ftc.teamcode.common.Angle;
//import org.firstinspires.ftc.teamcode.common.Distance;
//import org.firstinspires.ftc.teamcode.common.Position;
//import org.firstinspires.ftc.teamcode.Visual.PhoneManager;
//import java.io.PrintWriter;
//
//
//public class VuforiaPosition extends Visual {
//    public PhoneManager cameraManager = new PhoneManager();
//    private VuforiaSkyStone vuforia = new VuforiaSkyStone();
//
//    @Override
//    public void init() {
//        //initialize camera manager
//        cameraManager.startCaptureWithViews(telemetry, hardwareMap.appContext);
//        //cameraManager.startCapture(telemetry, hardwareMap.appContext);
//
//        //initialize vuforia sksytone
//        /*
//        vuforia.initialize(
//                // Vuforia License key (developer.vuforia.com):
//                "Ae7oRjb/////AAABmV3pkVnpEU9Pv3XaN0o2EZ5ttngvTMliTd5nX0843lAXhah50oPXg63sdsiK9/BFMjXkw9lMippdx4bHQo5kycWr1GcFcv+QlVNEpSclUqu9Zzj4FYVl+J2ScSAXSyuCRWMRWd3AikCfhAtlwFe7dnMIfpVniU8Yr8o3YumS2/5LjNU2wIkiJak5IHlnugT414wsrzyqemO63BHn0Olbi3REkd61RxW3cE4lbSts3OI0GfnT57/Nw6/YfLAZQ69eCz0eEckVjPmbt7evb8lYo5gEpzm+wf5LVPaAzZWVj/gSQywzPKA8zoz4q6hl4zuAd3647Y3smuWVI8PpQzRwt5vP8d07Qt39p+/zEOrcGRDo",
//                //VuforiaLocalizer.CameraDirection.BACK, // Phone back camera
//                hardwareMap.get(WebcamName.class, "Webcam 1"), // webcam hardware device
//                "teamwebcamcalibrations.xml", //the name of the webcam calibration file (Sent to RC using OnBotJava interface)
//                false, // extended tracking is not supported by this webcam
//                false, // Enable camera monitoring (the axes or teapot)
//                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // Show axes, not the teapot
//                0, 0, 0, // the position and angle of the phone on the robot
//                0, 0, 0,
//                true // Use the competition field layout
//        );
//        vuforia.activate();
//        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
//        */
//    }
//
//    @Override
//    public Position getPosition() {
//        double x = 0;
//        double y = 0;
//        double theta = 0;
//        int count = 0;
//
//        //find average position on the field according to each visible trackable
//        /*
//        for (String name : VuforiaSkyStone.TRACKABLE_NAMES) {
//            VuforiaBase.TrackingResults track = vuforia.track(name);
//            if (track.isVisible) {
//                x += track.x;
//                y += track.y;
//                theta += track.xAngle;
//                count++;
//            }
//        }
//        */
//
//        //debugging for now since the Distance class is not finished
//        //telemetry.addData("x", x);
//        //telemetry.addData("y", y);
//        //telemetry.addData("theta", theta);
//        //telemetry.addData("count", count);
//
//        //if there is nothing detected, return null
//        if (count == 0)
//            return null;
//        else
//            return new Position(Distance.fromMillimeters(x / count), Distance.fromMillimeters(y / count), Angle.fromDegrees(theta / count));
//    }
//
//    @Override
//    public findTeamElement findTeamElement() {
//        return null;
//    }
//
//    @Override
//    public double getTeamElementOffset() {
//        return 0;
//    }
//
//    public boolean[] isBlack(PrintWriter logging) {
//        while (!cameraManager.isNew);
//        Bitmap realFrame = cameraManager.getCurrentFrame().copy(Bitmap.Config.RGB_565, false);
//        cameraManager.isNew = false;
//        Bitmap currentFrame = Bitmap.createScaledBitmap(Bitmap.createBitmap(realFrame, 0, 0, 640, 240), 1, 1, false);
//        Bitmap currentFrame2 = Bitmap.createScaledBitmap(Bitmap.createBitmap(realFrame, 640, 0, 640, 240), 1, 1, false);
//
//        double[] hsv = new double[3];
//        PhoneManager.colorToHSV(currentFrame.getPixel(0, 0), hsv);
//        telemetry.addLine(hsv[0] + " " + hsv[1] + " " + hsv[2]);
//        double[] hsv2 = new double[3];
//        PhoneManager.colorToHSV(currentFrame2.getPixel(0, 0), hsv2);
//        telemetry.addLine(hsv2[0] + " " + hsv2[1] + " " + hsv2[2]);
//        telemetry.addLine(String.format("SATURATIONS! %f, %f", hsv[1], hsv2[1]));
//        logging.println(String.format("SATURATIONS! %f, %f", hsv[1], hsv2[1]));
//        return new boolean[] {hsv[1] < 0.6, hsv2[1] < 0.6};
//    }
//
//    public double getTeamElement() {
//        cameraManager.isNew = false;
//        while (!cameraManager.isNew) ;
//        Bitmap realFrame = cameraManager.getCurrentFrame().copy(Bitmap.Config.RGB_565, false);
//        //Bitmap currentFrame = Bitmap.createScaledBitmap(realFrame, 720, 10, false);
//        Bitmap currentFrame = Bitmap.createScaledBitmap(Bitmap.createBitmap(realFrame, 0, 120, 1280, 240), 320, 1, false);
//        telemetry.addData("Height", realFrame.getHeight());
//        telemetry.addData("Width", realFrame.getWidth());
//        double[] hsv = new double[3];
//        int lastYellow = -1; // keep track of the last yellow pixel seen
//        int blackBounds[] = {-1, -1};
//
//        // (DEBUG) show the skystone and black pixels a special color
//        for (int x = currentFrame.getWidth()/4; x < (currentFrame.getWidth() * 3/4); x++) {
//            PhoneManager.colorToHSV(currentFrame.getPixel(x, 0), hsv);
//            if (30 < hsv[0] && hsv[0] < 50 && hsv[1] > 0.7) {
//                if (lastYellow == -1 || x - lastYellow < 20) {
//                    lastYellow = x;
//                    currentFrame.setPixel(x, 0, 0x00FF00);
//                } else if (blackBounds[1] == -1){
//                    blackBounds[0] = lastYellow - 1;
//                    blackBounds[1] = x - 1;
//                    lastYellow = x;
//                }
//            } else if (lastYellow == -1) {
//                currentFrame.setPixel(x, 0, 0xFF00FF);
//            } else if (x - lastYellow < 20) {
//                currentFrame.setPixel(x, 0, 0x0000FF);
//            } else {
//                currentFrame.setPixel(x, 0, 0xFF0000);
//            }
//        }
//
//        telemetry.addData("Left:", blackBounds[0]);
//        telemetry.addData("Image Center", currentFrame.getWidth() / 2);
//        telemetry.addData("Skystone Center", (blackBounds[1] + blackBounds[0]) / 2);
//        telemetry.addData("Right", blackBounds[1]);
//
//
//        //cameraManager.updatePreviewBitmap(currentFrame);
//
//
//        // Now we need a proportion of left to right... Just subtracting might work...
//
//        return ((blackBounds[1] + blackBounds[0]) / 2) - (currentFrame.getWidth() / 2);
//    }
//
//
//    public findTeamElement findSkystone() {
//        Bitmap currentFrame = cameraManager.getCurrentFrame().copy(Bitmap.Config.RGB_565, true);
//        Bitmap scaledFrame = Bitmap.createScaledBitmap(
//                Bitmap.createBitmap(currentFrame, 0, 240, 1280, 420), 100, 50, false);
//
//        //find skystone
//        //counts the black pixels in a 3*3 area
//        double[] hsv = new double[3];
//        double[] blackPixels = new double[3];
//        for (int i = 0; i < 3; i++) {
//            for (int x = 0; x < 3; x++)
//                for (int y = 0; y < 3; y++) {
//                    PhoneManager.colorToHSV(scaledFrame.getPixel(27 + i * 25 + x, 4 + y), hsv);
//
//                    blackPixels[i] += hsv[1];
//                }
//        }
//
//        telemetry.addData("right", blackPixels[0]);
//        telemetry.addData("center", blackPixels[1]);
//        telemetry.addData("left", blackPixels[2]);
//
//        //set other stones magenta
//        scaledFrame.setPixel(27, 5, 0xFF00FF);
//        scaledFrame.setPixel(52, 5, 0xFF00FF);
//        scaledFrame.setPixel(77, 5, 0xFF00FF);
//
//        findTeamElement result;
//
//        //set the blacker stone green
//        if (blackPixels[0] < blackPixels[1]) {
//            if (blackPixels[0] < blackPixels[2]) {
//                scaledFrame.setPixel(27, 5, 0x00FF00);
//                result = findTeamElement.Right;  //1st stone
//            } else {
//                scaledFrame.setPixel(77, 5, 0x00FF00);
//                result = findTeamElement.Left;  //3nd stone
//            }
//        } else if (blackPixels[1] < blackPixels[2]) {
//            scaledFrame.setPixel(52, 5, 0x00FF00);
//            result = findTeamElement.Center;  //2nd stone
//        } else {
//            scaledFrame.setPixel(77, 5, 0x00FF00);
//            result = findTeamElement().Left;  //3rd stone
//        }
//
//        //display the modified bitmap
//        cameraManager.updatePreviewBitmap(scaledFrame);
//        return result;
//    }
//
//    public void stop() {
//        cameraManager.stopCapture();
//    }
//}