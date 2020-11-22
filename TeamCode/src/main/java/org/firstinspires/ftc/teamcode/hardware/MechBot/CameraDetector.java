package org.firstinspires.ftc.teamcode.hardware.MechBot;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

//import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
//import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

/**
 * Swerve chassis consists of 4 wheels with a Servo and DcMotor attached to each.
 * Track (distance between left and right wheels), wheelbase (distance between front and back wheels)
 * and wheel radius are adjustable.
 * Expected hardware configuration is:<br />
 * Servos: servoFrontLeft, servoFrontRight, servoBackLeft, servoBackRight.<br />
 * Motors: motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight.<br />
 * Orientation sensors (optional): imu, imu2
 */
public class CameraDetector extends Logger<CameraDetector> implements Configurable {
    static Logger<CameraDetector> logger = new Logger<>();

    static {
        logger.configureLogging("CameraStoneDetector", Log.VERBOSE);
    }

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AS0FKrL/////AAABmTcBCNs1gE8uh4tntGA7HSgXRT5npDQpV2pw5tlqbZCI6WJQRf0bKf458A218bGkQJCWkJzvJy6UtNnhziraRVDDZSnTSZGSD7s3WN9jNYqBiSoO3CVE6FU2dX1yuJNa1zfiEhcGT8ChTd+kucE/q3sXsy/nw1KqlW/7uEaEeRwaCPseqsbNrc1HZ1bi18PzwQpCaypDruqqVEyZ3dvTqDmjPg7WFBe2kStPR/qTtcLSXdE804RxxkgTGUtDMIG7TTbAdirInGVZw2p2APZKAQdYofYW2E0Ss5hZCeL55zflSuQK0QcW1sAyvaTUMd/fDse4FgqxhnfK0ip0Kc+ZqZ6XJpof/Nowwxv3IgDWZJzO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private AdjustableServo camLR; // webcam left/right servo
    private Telemetry tl;
    private double camPos = 0;

    private double stoneYpos = 250;//390

    public final double CAM_MIN = 0.01;
    public final double CAM_MAX = 0.99;
    public final double CAM_INIT = 0.45;
    public final double CAM_BLUE_IN = 0.56;
    public final double CAM_BLUE_OUT = 0.22;
    public final double CAM_RED_IN = 0.22;
    public final double CAM_RED_OUT = 0.56;
    public final double CAM_TELE_OP = 0.45;

    //multipliers for alternative detection
    double[][] relativePointsQuad = new double[][]{{0,-50}, {-50,-30}, {+50, -30}, {-50,20}, {+50,20}, {-50,-60}, {0, -67}, {+50,-60}, {-85, -60}, {+85, -60}, {-85, 0}, {+85, 0}, {-50,50}, {+50,50}, {0, -95}};
    double[][] multipliersQuad = new double[][]{{-0.2, -0.2, -0.1}, {0.7, 0.5, -0.5}, {0.7, 0.5, -0.5}, {0.7, 0.5, -0.5}, {0.7, 0.5, -0.5}, {0.8, 0.6, -0.2}, {0.8, 0.6, -0.2}, {0.8, 0.6, -0.2}, {-0.2, -0.2, -0.1}, {-0.2, -0.2, -0.1}, {-0.2, -0.2, -0.1}, {-0.2, -0.2, -0.1}, {-0.2, -0.2, -0.1}, {-0.2, -0.2, -0.1}, {-0.2, -0.2, -0.1}};
    double[][] relativePointsSingle = new double[][]{{-40,-35}, {40,-35}, {-50, 10}, {+50, 10}, {-45,55}, {+45,55}, {-95, 5}, {95, 5}};
    double[][] multipliersSingle = new double[][]{{-0.1, -0.1, -0.1}, {-0.1, -0.1, -0.1}, {0.8, 0.6, -0.2}, {0.8, 0.6, -0.2}, {-0.3, -0.3, -0.3}, {-0.3, -0.3, -0.3}, {-0.3, -0.3, -0.3}, {-0.1, -0.1, -0.1}};
    double filterRatio = 4.0/3.0;

    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    VuforiaTrackables targetsUltimateGoal;
    public List<VuforiaTrackable> allTrackables;
    public OpenGLMatrix lastLocation;

    private static final float qField = 900; // mm
    private static final float hField = 1800; // mm
    private static final float fField = 3600; // mm
    private static final float mmPerInch        = 25.4f;
    private static final float mmPerCm          = 10.0f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor


    private class Filter //recognition class
    {
        public double[][] pixelPos;
        public double[][] multipliers;
        public Filter(double[][] pixelPos_, double[][] multipliers_, double shrinkRatio)
        {
            pixelPos = pixelPos_;
            for(int i = 0;i<pixelPos.length;i++)
            {
                pixelPos[i][0] /= shrinkRatio;
                pixelPos[i][1] /= shrinkRatio;
            }
            multipliers = multipliers_;
        }
        public double getScalar(Bitmap img, int x, int y) //filter score
        {
            double scalar = 0;
            for(int i = 0;i<pixelPos.length;i++)
            {
                int[] curr = getPixelValue(img,(int)(pixelPos[i][0]+x),(int)(pixelPos[i][1]+y)); //get pixel value
                double currPixelMultiplier = //do multipliers
                        ((double)(curr[0])*multipliers[i][0])
                        +((double)(curr[1])*multipliers[i][1])
                        +((double)(curr[2])*multipliers[i][2]);
                scalar += currPixelMultiplier; //add
            }
            return scalar;
        }
        public int[] getPixelValue(Bitmap img, int x, int y) //tbh this is just stackoverflow code because there isn't java.awt.Color
        {
            int color = img.getPixel(x,y);
            int r = (color >> 16) & 0xff;
            int g = (color >> 8) & 0xff;
            int b = color & 0xff;
            return new int[]{r, g, b};
        }
    }

    //    private CameraSystem camSys;
    private String lastError;

    @Override
    public void setAdjustmentMode(boolean on) {
        // CameraStoneDetector doesn't need an adjustment mode
        // Method is only declared for completeness of subsystem
    }

    @Override
    public String getUniqueName() {
        return "CameraStackDetector";
    }

    public TFObjectDetector getTfod() {
        return tfod;
    }

    public double getStoneYpos() {
        return stoneYpos;
    }

    public void set_cam_pos(double pos) {
        if (camLR==null) return;
        camLR.setPosition(pos);
        camPos = pos;
    }

    public double getCamPos() { return camPos; }
    public void inc_cam_pos() {
        if (camLR==null) return;
        double pos = Math.min(camPos+0.01, CAM_MAX);
        set_cam_pos(pos);
    }

    public void dec_cam_pos() {
        if (camLR==null) return;
        double pos = Math.max(camPos-0.01, CAM_MIN);
        set_cam_pos(pos);
    }

    public void configure(Configuration configuration, boolean isTFOD) {
        logger.verbose("Start Configuration");
        /*
         * InitVuforia(): Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        camLR = new AdjustableServo(0, 1).configureLogging(
                logTag + ":camLR", logLevel
        );
        camLR.configure(configuration.getHardwareMap(), "camLR");
        configuration.register(camLR);
        set_cam_pos(CAM_INIT);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = configuration.getHardwareMap().get(WebcamName.class, "Webcam");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        // initTfod()

        if(isTFOD)
        {
            int tfodMonitorViewId = configuration.getHardwareMap().appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", configuration.getHardwareMap().appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

            tfodParameters.minResultConfidence = 0.6f;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);


            logger.verbose("CameraStackDetector status: %s", tfod);

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                logger.verbose("Start tfod Activation");
                tfod.activate();
                logger.verbose("tfod activate: ", tfod);

                tfod.setZoom(2.0, (16.0 / 9.0));
            }
        }
        else
        {
            vuforia.enableConvertFrameToBitmap();
            targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
            VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
            blueTowerGoalTarget.setName("Blue Tower Goal Target");
            VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
            redTowerGoalTarget.setName("Red Tower Goal Target");
            VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
            redAllianceTarget.setName("Red Alliance Target");
            VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
            blueAllianceTarget.setName("Blue Alliance Target");
            VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
            frontWallTarget.setName("Front Wall Target");
            // For convenience, gather together all the trackable objects in one easily-iterable collection */
            allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(targetsUltimateGoal);

            redAllianceTarget.setLocation(OpenGLMatrix
                    .translation(0, -hField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            blueAllianceTarget.setLocation(OpenGLMatrix
                    .translation(0, hField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
            frontWallTarget.setLocation(OpenGLMatrix
                    .translation(-hField, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

            // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
            blueTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(hField, qField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
            redTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(hField, -qField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//            blue1.setLocation(OpenGLMatrix
//                    //.translation(-quadField, halfField, mmTargetHeight)
//                    .translation(qField, fField, mmTargetHeight)
//                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));


            // setup Camera rotation
            phoneYRotate = -90; // camera facing forward
            phoneXRotate = 0; // in case of Portrait mode, phoneXRotate = 90;

            final float CAMERA_FORWARD_DISPLACEMENT = 7.5f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
            final float CAMERA_VERTICAL_DISPLACEMENT = 7.5f * mmPerInch;   // eg: Camera is 8 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
            }
            targetsUltimateGoal.activate();
        }

        // register CameraStoneDetector as a configurable component
        configuration.register(this);
    }

    private Bitmap convertFrameToBitmap(VuforiaLocalizer.CloseableFrame frame) {
        long numImages = frame.getNumImages();
        Image image = null;

        for (int imageI = 0; imageI < numImages; imageI++) {
            if (frame.getImage(imageI).getFormat() == PIXEL_FORMAT.RGB565) {
                image = frame.getImage(imageI);
                break;
            }
        }
        if (image == null) {
            for (int imageI = 0; imageI < numImages; imageI++) {
                //For diagnostic purposes
            }
            lastError = "Failed to get RGB565 image format!";
            return null;
        }
        Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(image.getPixels());
        return bitmap;
    }

    public Bitmap captureVuforiaBitmap(/*double xOffsetF, double yOffsetF, double widthF, double heightF*/) {
        Bitmap bitmapTemp = null;
        lastError = "";
        int capacity = vuforia.getFrameQueueCapacity();
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().poll();

        if (frame == null) {
            lastError = "ERROR! Failed to retrieve frame!";
            return null;
        }

        bitmapTemp = convertFrameToBitmap(frame);

        frame.close();
        if (bitmapTemp == null) {
            lastError = "ERROR! Failed to retrieve bitmap";
            return null;

        }
        //White Balance applied here
//        int whitestPixel = getWhitestPixel(bitmapTemp);
//        applyWhiteBalance(bitmapTemp, whitestPixel);

        //Bitmap bitmap = cropBitmap(bitmapTemp, xOffsetF, yOffsetF, widthF, heightF);
        Bitmap bitmap = bitmapTemp;
        return bitmap;
    }

    public ToboSigma.SkystoneLocation getSkystonePositionElementary(Telemetry tl, boolean debug, Robot2.ProgramType teamColor) {
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);//used to be PIXEL_FORMAT.RGB565
        vuforia.enableConvertFrameToBitmap();
        VuforiaLocalizer.CloseableFrame frm = null;
        try {
            frm = vuforia.getFrameQueue().take();
        } catch (InterruptedException e) {
            tl.addLine(e.toString());
        }
//        tl.addData("NumImage", frm.getNumImages());
        long numImages = frm.getNumImages();
        Image img = null;
        for (int i = 0; i < numImages; i++) {
            if (frm.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {//PIXEL_FORMAT.RGB565
                img = frm.getImage(i);
                break;
            }
        }

        int width = img.getWidth();
        int height = img.getHeight();
//        if (debug) {
//            tl.addData("image Width", width);
//            tl.addData("image Height", height);
//        }
        Bitmap bitmap = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);//Bitmap.Config.RGB_565
        bitmap.copyPixelsFromBuffer(img.getPixels());

        long blackXsum = 0;
        long blackCount = 0;

        String path = Environment.getExternalStorageDirectory().toString();
//        PrintWriter pw = null;
//        try {
//            pw = new PrintWriter(new FileWriter(new File(path, "color5.txt")));
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
        tl.addData("stoneYpos", stoneYpos);
        for (int i = 34; i < width-34; i++) {
            int color = bitmap.getPixel(i, (int) stoneYpos);//was fixed at 280
//            int R = (int) (((color >> 11) & 0x1F) * 255.0 / 31.0 + 0.5);
//            int G = (int) (((color >> 5) & 0x3F) * 255.0 / 63.0 + 0.5);
//            int B = (int) ((color & 0x1F) * 255.0 / 31.0 + 0.5);
//            int r = ((((color >> 11) & 0x1F) * 527) + 23) >> 6;
//            int g = ((((color >> 5) & 0x3F) * 259) + 33) >> 6;
//            int b = (((color & 0x1F) * 527) + 23) >> 6;
            int r = (color >> 16) & 0xFF;
            int g = (color >> 8) & 0xFF;
            int b = color & 0xFF;
//            pw.println(String.format("r %d, g %d, b%d, at x=%d", r, g, b, i));
            int brightness = r + g + b;
            if (brightness < 80) {
                blackCount++;
                blackXsum += i;
//                pw.println(String.format("r %d, g %d, b%d, at x=%d", r, g, b, i));
            }
        }
//        pw.flush();
//        pw.close();
        tl.addData("black count", blackCount);
        tl.addData("black X sum", blackXsum);

        if (debug) {
            try {
                OutputStream fOut = new FileOutputStream(new File(path, "ss2-14-1.png"));
                bitmap.compress(Bitmap.CompressFormat.PNG, 100, fOut); // bmp is your Bitmap instance
                // PNG is a lossless format, the compression factor (100) is ignored
                fOut.flush(); // Not really required
                fOut.close(); // do not forget to close the stream
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if (debug)
            tl.update();

        if (blackCount == 0 || blackCount > 250) {
            return ToboSigma.SkystoneLocation.UNKNOWN;
        }
        long blackAvg = blackXsum / blackCount;
        if (blackAvg < 213) {
            return ToboSigma.SkystoneLocation.LEFT;
        } else if (blackAvg > 426) {
            return ToboSigma.SkystoneLocation.RIGHT;
        } else {
            return ToboSigma.SkystoneLocation.CENTER;
        }
    }

    public ToboMech.TargetZone getTargetZoneAlternative() throws InterruptedException {
        // vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.enableConvertFrameToBitmap();
        VuforiaLocalizer.CloseableFrame frm;

        frm = vuforia.getFrameQueue().take();

        int numImages = (int) frm.getNumImages();
        Image img = null;

        img = frm.getImage((int) (numImages-1));

        if (img==null) return ToboMech.TargetZone.UNKNOWN;

        Bitmap bitmap = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(img.getPixels());

        //vuforia stuff ^

        //init filters
        Filter quadStack = new Filter(relativePointsQuad, multipliersQuad, filterRatio);
        Filter singleStack = new Filter(relativePointsSingle, multipliersSingle, filterRatio);

        double maxQuadScalar = -99999;
        int startX = 251; int startY = 270;
        int scanSpacing = 30;
        int zeroValueQuad =  -600; double percentValueQuad = 4.0; //maps values to 0-100
        //scans 15 points around the desired area
        for(int y = 0;y<3;y++)
        {
            for(int x = 0;x<5;x++)
            {
                double temp = quadStack.getScalar(bitmap, startX+(x*scanSpacing), startY + (y*scanSpacing));
                if(temp>maxQuadScalar)
                {
                    maxQuadScalar = temp;
                }
            }
        }
        maxQuadScalar += zeroValueQuad;
        //maxQuadScalar /= percentValueQuad;

        double maxSingleScalar = -99999;
        int zeroValueSingle = 144; double percentValueSingle = 2.05;
        for(int y = 0;y<3;y++)
        {
            for(int x = 0;x<5;x++)
            {
                double temp = singleStack.getScalar(bitmap, startX+(x*scanSpacing), startY + (y*scanSpacing));
                if(temp>maxSingleScalar)
                {
                    maxSingleScalar = temp;
                }
            }
        }

        maxSingleScalar += zeroValueSingle;
        maxSingleScalar /= percentValueSingle;
        if (tl!=null) {
            tl.addData("maxSingleScalar", "%5.0f (#-img=%2d)", maxSingleScalar,numImages);
            tl.addData("maxQuadScalar  ", "%5.0f", maxQuadScalar);
            // tl.update();
        }

        if(maxQuadScalar<20)
        {
            return ToboMech.TargetZone.ZONE_A;
        }
        else
        if(maxQuadScalar>maxSingleScalar)
        {
            return ToboMech.TargetZone.ZONE_C;
        }
        else
        if(maxSingleScalar>maxQuadScalar)
        {
            return ToboMech.TargetZone.ZONE_B;
        }
        else
        {
            return ToboMech.TargetZone.ZONE_A;
        }
    }

    public ToboMech.TargetZone getTargetZone() {

        ToboMech.TargetZone rings = ToboMech.TargetZone.UNKNOWN;

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.startTime();

        if (tfod == null)
        {
            return ToboMech.TargetZone.UNKNOWN;
        }

        int detectedRings = 0; //0 = none, 1 = single, 2 = quad

        while (elapsedTime.seconds() < 0.3 && rings == ToboMech.TargetZone.UNKNOWN)
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if(updatedRecognitions == null || updatedRecognitions.size()<1)
            {
                continue;
            }
            for(Recognition recognition : updatedRecognitions)
            {
                if(detectedRings == 4 && updatedRecognitions.size()>1)
                {
                    break; //if we detect more than one and it's a quad already, most likely it's a quad
                }

                if(recognition.getLabel()=="Single") {
                    // sometime the quad-stack is retuned as single-stack. Use the height to decide if it is single (height<80) or quad-stack
                    if (Math.abs(recognition.getBottom()-recognition.getTop())<80)
                        detectedRings = 1;
                    else
                        detectedRings = 4;
                }
                else {
                    detectedRings = 4;
                }
            }
            switch(detectedRings)
            {
                case 1:
                    rings = ToboMech.TargetZone.ZONE_B;
                    break;
                case 4:
                    rings = ToboMech.TargetZone.ZONE_C;
                    break;
                default:
                    rings = ToboMech.TargetZone.ZONE_A;
            }
        }
        if (rings== ToboMech.TargetZone.UNKNOWN)
            rings= ToboMech.TargetZone.ZONE_A;
        return rings;
    }

    public ToboSigma.SkystoneLocation getSkystonePositionTF(boolean redSide) {
        return getSkystonePositionTF(redSide, true);
    }

        public ToboSigma.SkystoneLocation getSkystonePositionTF(boolean redSide, boolean forSQT) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        int left_center_border_x = 200;
        int center_right_border_x = 400;

        int min_stone_width = 150;
        int max_stone_width = 320;

        int n_ss = 0;
        int n_rs = 0;
        double skystone_width = 0;
        double skystone_pos = 0;
        double rstone_width[] = new double[2];
        double rstone_pos[] = new double[2];

        logger.verbose("Start getGoldPositionTF()");

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.startTime();

        ToboSigma.SkystoneLocation skystoneLocation = ToboSigma.SkystoneLocation.UNKNOWN;
        //int goldXCoord = -1;
        //int silverXCoord = -1;
        if (tfod == null) {
            return ToboSigma.SkystoneLocation.UNKNOWN;
        }

        while (elapsedTime.seconds() < 0.3 && skystoneLocation == ToboSigma.SkystoneLocation.UNKNOWN) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            n_ss = n_rs = 0;
            if (updatedRecognitions == null || updatedRecognitions.size() < 1) {
                continue;
            }
            //logger.verbose("Starting recognitions");
            //logger.verbose("Recognitions: %d", (int) updatedRecognitions.size());
            for (Recognition recognition :
                    updatedRecognitions) {
                double width = recognition.getRight() - recognition.getLeft();
                if (width>min_stone_width) {
                    stoneYpos = 0.6 * recognition.getBottom() + 0.4 * recognition.getTop();
                }
                if (width < max_stone_width && width > min_stone_width) {
                    if (recognition.getLabel() == "Stone") {
                        if (n_rs < 2) {
                            rstone_width[n_rs] = recognition.getRight() - recognition.getLeft();
                            rstone_pos[n_rs] = (recognition.getRight() + recognition.getLeft()) / 2;
                        }
                        n_rs++;
                    } else {
                        skystone_width = recognition.getRight() - recognition.getLeft();
                        skystone_pos = (recognition.getRight() + recognition.getLeft()) / 2;
                        n_ss++;
                    }
                }
            }
            //logger.verbose("Valid recognitions: %d", validRecognitions);
            if (n_ss != 1 && n_rs != 2) {
                continue;
            }
            int ss_pos_on_screen = 3; // 0=left, 1=center, 2=right, 3=unknown
            if (n_ss != 1) { // use regular two stones to imply the location of SS
                boolean rstone_map[] = new boolean[3];
                for (int i = 0; i < 3; i++) rstone_map[i] = false;
                for (int i = 0; i < n_rs; i++) {
                    if (rstone_pos[i] < left_center_border_x)
                        rstone_map[0] = true;
                    else if (rstone_pos[i] > center_right_border_x) {
                        rstone_map[2] = true;
                    } else if (rstone_pos[i] >= left_center_border_x && rstone_pos[i] <= center_right_border_x) {
                        rstone_map[1] = true;
                    }
                }
                for (int i = 0; i < 3; i++) {
                    if (rstone_map[i] == false) {
                        ss_pos_on_screen = i;
                        break;
                    }
                }
            } else {
                if (skystone_pos < left_center_border_x) {
                    ss_pos_on_screen = 0;
                } else if (skystone_pos > center_right_border_x) {
                    ss_pos_on_screen = 2;
                } else if (skystone_pos >= left_center_border_x && skystone_pos <= center_right_border_x) {
                    ss_pos_on_screen = 1;
                } else {
                    ss_pos_on_screen = 3;
                }
            }
            if(forSQT) {
                if (redSide) {
                    if (ss_pos_on_screen == 0) {
                        skystoneLocation = ToboSigma.SkystoneLocation.RIGHT;
                    } else if (ss_pos_on_screen == 2) {
                        skystoneLocation = ToboSigma.SkystoneLocation.CENTER;
                    } else if (ss_pos_on_screen == 1) {
                        skystoneLocation = ToboSigma.SkystoneLocation.LEFT;
                    } else {
                        skystoneLocation = ToboSigma.SkystoneLocation.UNKNOWN;
                    }
                } else {
                    if (ss_pos_on_screen == 0) {
                        skystoneLocation = ToboSigma.SkystoneLocation.CENTER;
                    } else if (ss_pos_on_screen == 2) {
                        skystoneLocation = ToboSigma.SkystoneLocation.LEFT;
                    } else if (ss_pos_on_screen == 1) {
                        skystoneLocation = ToboSigma.SkystoneLocation.RIGHT;
                    } else {
                        skystoneLocation = ToboSigma.SkystoneLocation.UNKNOWN;
                    }
                }
            }
            else
            {
                if (ss_pos_on_screen == 0) {
                    skystoneLocation = ToboSigma.SkystoneLocation.LEFT;
                } else if (ss_pos_on_screen == 2) {
                    skystoneLocation = ToboSigma.SkystoneLocation.RIGHT;
                } else if (ss_pos_on_screen == 1) {
                    skystoneLocation = ToboSigma.SkystoneLocation.CENTER;
                } else {
                    skystoneLocation = ToboSigma.SkystoneLocation.UNKNOWN;
                }
            }
        }

//        if (tfod != null) {
//            tfod.deactivate();
//            tfod.shutdown();
//            logger.verbose("Tfod shutdown", tfod);
//        }

        switch (skystoneLocation) {
            case LEFT:
                logger.verbose("SampleLocation: Left");
                break;
            case RIGHT:
                logger.verbose("SampleLocation: Right");
                break;
            case CENTER:
                logger.verbose("SampleLocation: Center");
                break;
            case UNKNOWN:
                logger.verbose("Sample Location: Unknown");
                break;
            default:
                logger.verbose("Sample Location: Unknown");
                break;
        }
        return skystoneLocation;
    }

    public void SSLocTest(double[] sslocation) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        //int left_center_border_x = 200;
        //int center_right_border_x = 400;

        //int min_stone_width = 150;
        //int max_stone_width = 250;
        //int large_stone_width = 320;

        logger.verbose("Start getGoldPositionTF()");

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.startTime();

        //int goldXCoord = -1;
        //int silverXCoord = -1;
        sslocation[0] = sslocation[1] = -1;

        while (elapsedTime.seconds() < 0.2 && sslocation[0] == -1) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions == null || updatedRecognitions.size() < 1) {
                continue;
            }
            //logger.verbose("Starting recognitions");
            //logger.verbose("Recognitions: %d", (int) updatedRecognitions.size());
            int validRecognitions = 0;
            /*
            for (Recognition recognition :
                    updatedRecognitions) {
                double width = recognition.getRight() - recognition.getLeft();
                if (width < max_stone_width && width > min_stone_width) {
                    validRecognitions++;
                }
            }
            */
            //logger.verbose("Valid recognitions: %d", validRecognitions);
            if (validRecognitions < 1 || validRecognitions > 3) {
                continue;
            }
            for (Recognition recognition :
                    updatedRecognitions) {
                if (recognition.getLabel() == "Stone") {
                    continue;
                }
                //double pos = (recognition.getRight() + recognition.getLeft()) / 2;
                //double skystone_width = recognition.getRight() - recognition.getLeft();
                sslocation[0] = recognition.getLeft();
                sslocation[1] = recognition.getRight();

            }
        }
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
            logger.verbose("Tfod shutdown", tfod);
        }
        /*
        switch (skystoneLocation) {
            case LEFT:
                logger.verbose("SampleLocation: Left");
                break;
            case RIGHT:
                logger.verbose("SampleLocation: Right");
                break;
            case CENTER:
                logger.verbose("SampleLocation: Center");
                break;
            case UNKNOWN:
                logger.verbose("Sample Location: Unknown");
                break;
            default:
                logger.verbose("Sample Location: Unknown");
                break;
        }
         */
    }

    public double[] getPositionFromVuforia()
    {
        double x = -1;
        double y = -1;
        double heading = -1;
        boolean targetVisible;
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            //telemetry.addData("Pos (cm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
            x = translation.get(0) / 10;
            y = translation.get(1) / 10;

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            heading = rotation.thirdAngle;
            //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }

        return new double[]{x,y,heading};
        //telemetry.update();
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        tl = telemetry;
        Telemetry.Line line = telemetry.addLine();
        if (camLR!=null) {
            line.addData("WebCam", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return camPos;
                }
            });
        }
    }

    public void end() {
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
            logger.verbose("Tfod shutdown", tfod);
        }
    }
}


