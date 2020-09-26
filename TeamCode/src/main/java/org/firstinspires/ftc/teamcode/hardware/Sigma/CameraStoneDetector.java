package org.firstinspires.ftc.teamcode.hardware.Sigma;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.List;
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
public class CameraStoneDetector extends Logger<CameraStoneDetector> implements Configurable {
    static Logger<CameraStoneDetector> logger = new Logger<>();

    static {
        logger.configureLogging("CameraStoneDetector", Log.VERBOSE);
    }

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AS0FKrL/////AAABmTcBCNs1gE8uh4tntGA7HSgXRT5npDQpV2pw5tlqbZCI6WJQRf0bKf458A218bGkQJCWkJzvJy6UtNnhziraRVDDZSnTSZGSD7s3WN9jNYqBiSoO3CVE6FU2dX1yuJNa1zfiEhcGT8ChTd+kucE/q3sXsy/nw1KqlW/7uEaEeRwaCPseqsbNrc1HZ1bi18PzwQpCaypDruqqVEyZ3dvTqDmjPg7WFBe2kStPR/qTtcLSXdE804RxxkgTGUtDMIG7TTbAdirInGVZw2p2APZKAQdYofYW2E0Ss5hZCeL55zflSuQK0QcW1sAyvaTUMd/fDse4FgqxhnfK0ip0Kc+ZqZ6XJpof/Nowwxv3IgDWZJzO";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private double stoneYpos = 250;//390

    //    private CameraSystem camSys;
    private String lastError;

    @Override
    public void setAdjustmentMode(boolean on) {
        // CameraStoneDetector doesn't need an adjustment mode
        // Method is only declared for completeness of subsystem
    }

    @Override
    public String getUniqueName() {
        return "CameraStoneDetector";
    }

    public TFObjectDetector getTfod() {
        return tfod;
    }

    public double getStoneYpos() {
        return stoneYpos;
    }


    public void configure(Configuration configuration, ToboSigma.CameraSource cameraSource) {
        logger.verbose("Start Configuration");
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        if (cameraSource == ToboSigma.CameraSource.WEBCAM_RIGHT) {
            // parameters.cameraName = configuration.getHardwareMap().get(WebcamName.class, "WebcamRight");
            parameters.cameraName = configuration.getHardwareMap().get(WebcamName.class, "Webcam");
        } else if (cameraSource == ToboSigma.CameraSource.WEBCAM_LEFT) {
            // parameters.cameraName = configuration.getHardwareMap().get(WebcamName.class, "WebcamLeft");
            parameters.cameraName = configuration.getHardwareMap().get(WebcamName.class, "Webcam");
        } else {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        }

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        int tfodMonitorViewId = configuration.getHardwareMap().appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", configuration.getHardwareMap().appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        logger.verbose("CameraStoneDetector status: %s", tfod);

        tfodParameters.minimumConfidence = 0.6;

        if (cameraSource == ToboSigma.CameraSource.INTERNAL)
            com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);
//        com.vuforia.CameraDevice.getInstance().setField("iso", "800");

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            logger.verbose("Start tfod Activation");
            tfod.activate();
            logger.verbose("tfod activate: ", tfod);
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

    public ToboSigma.SkystoneLocation getSkystonePositionElementary(Telemetry tl, boolean debug, ToboSigma.AutoTeamColor teamColor) {
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
}

/**
 * Set up telemetry lines for chassis metrics
 * Shows current motor power, orientation sensors,
 * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
 * and servo position for each wheel
 */
//    public void setupTelemetry(Telemetry telemetry) {
//        Telemetry.Line line = telemetry.addLine();
//        if ()
//            line.addData("CameraStoneDetector", "Recog. Count= %d", new Func<Integer>() {
//                @Override
//                public Integer value() {
//
//                }
//            });
//    }
