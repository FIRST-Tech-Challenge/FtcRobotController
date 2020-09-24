package com.hfrobots.tnt.season1819;

import android.speech.tts.TextToSpeech;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Queue;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class TensorflowThread extends Thread {
    public enum GOLD_MINERAL_POSITION {
        LEFT,
        CENTER,
        RIGHT;
    }

    // TODO: Explain what a queue is and why we're using it (might need to show caller's use of
    // it to do so!
    private final Queue<GOLD_MINERAL_POSITION> tfResultsQueue;

    private final long timeoutMillis;

    private long threadStartedTimeMillis = 0;

    private final HardwareMap hardwareMap;

    private final TextToSpeech textToSpeech;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AdD5Ysf/////AAABmZcK/df7dUf0k4rDpBhNTldjBolKypT1w5SexGrFThbu+AGLdJMYnHfmjyS6s8jkuXXbJQK0FOywlbOJKqKtpz046IE5sPjPyMpsOQ0BXEaU6digujkChlecDRd8VTMrICSNrVfU8EViUzqiB1PSpHxsMXkRUkBbWua0SYDUYU3ZZrxjZC2T9FfrTcsFW1EpQOpda7+LVIeF1DjOm6y5yPqrzrOD5XB/3ZC0s+zcDxRxzFFSxO8ziEYlhNWkMge7spSKmVeDeTRx0Fozgj0x/seiCxKWx/mrgoNJMtDAqTabMRpP8PE+IDQlr9jheiDlTgCuYktpyFHbrhLrjAh5gLyz/S5oeyfwm/Fg2zSMZqgb";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public TensorflowThread(final Queue<GOLD_MINERAL_POSITION> tfResultsQueue,
                            final HardwareMap hardwareMap,
                            final long timeoutMillis) {
        this.tfResultsQueue = tfResultsQueue;
        this.timeoutMillis = timeoutMillis;
        this.hardwareMap = hardwareMap;

        textToSpeech = new TextToSpeech(hardwareMap.appContext,
                new TextToSpeech.OnInitListener()
                {
                    @Override
                    public void onInit(int status)
                    {
                        if (status != TextToSpeech.ERROR)
                        {
                            textToSpeech.setLanguage(Locale.ENGLISH);
                        }
                    }
                });
    }

    public void initDetection() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        initTfod();
    }

    @Override
    public void run() {
        Log.d(LOG_TAG, "Tensorflow detection thread running");

        threadStartedTimeMillis = System.currentTimeMillis();

        if (tfod != null) {
            tfod.activate();
        }

        try {
            while (!isTimedOut()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        Log.d(LOG_TAG, "# Object Detected: " + updatedRecognitions.size());

                        findGoldMineral(updatedRecognitions);

                        if (!tfResultsQueue.isEmpty()) {
                           break; // leave the loop, we're done!
                        }
                    }
                }
            }
        } finally {
            try {
                shutdownTensorFlow();
            } catch (Throwable t) {
                Log.e(LOG_TAG, "Error when shutting down TensorFlow", t);
            }
        }

        try {
            if (tfResultsQueue.isEmpty()) {
                Log.d(LOG_TAG, "Mineral not seen");
                textToSpeech.speak("Error four oh four mineral not found ", TextToSpeech.QUEUE_FLUSH, null);
            }
        } finally {
            try {
                textToSpeech.shutdown();
            } catch (Throwable t) {
                Log.e(LOG_TAG, "Error when shutting down TTS", t);
            }
        }
    }

    public void shutdownTensorFlow() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void findGoldMineral(List<Recognition> updatedRecognitions) {
        if (updatedRecognitions.size() == 2 || updatedRecognitions.size() == 3) {
            int goldMineralX = -1;
            int goldMineralY = - 1;

            int silverMineral1X = -1;
            int silverMineral1Y = -1;

            int silverMineral2X = -1;
            int silverMineral2Y = -1;

            int mineralsFound = 0;

            for (Recognition recognition : updatedRecognitions) {
                Log.d(LOG_TAG, "TF recognized object labeled " + recognition.getLabel()
                + " @ " + recognition.getLeft() + ", " + recognition.getBottom()
                        + "conf: " + recognition.getConfidence());

                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    goldMineralX = (int) recognition.getLeft();
                    goldMineralY = (int) recognition.getBottom();

                    mineralsFound++;
                } else if (silverMineral1X == -1) { // This is from example, it could be more accurate, how?
                    silverMineral1X = (int) recognition.getLeft();
                    silverMineral1Y = (int) recognition.getBottom();

                    mineralsFound++;
                } else { // This is from example, it could be more accurate, how?
                    silverMineral2X = (int) recognition.getLeft();
                    silverMineral2Y = (int) recognition.getBottom();

                    mineralsFound++;
                }
            }

            Log.d(LOG_TAG, "Detected Mineral Positions (gold, silver, silver): ("
                    + goldMineralX + ", " + goldMineralY + ")"
            + ", (" + silverMineral1X + ", " + silverMineral1Y + "), ("
                    + silverMineral2X + ", " + silverMineral2Y + ")");

            // FIXME: Do we think "one and done" detection is good enough, or
            //        what if we detect that the gold mineral "moved"? What should
            //        the robot consider the position to be then?

            if (mineralsFound == 2) {
                if (goldMineralX !=-1) { // we have the gold mineral, where is it?
                    final int silverMineralX;

                    // FIXME: Do we want to know if sm2x is != -1? If not, why? (is it an invariant?)

                    silverMineralX = silverMineral1X;

                    if (silverMineralX > goldMineralX) {
                        // gold is on the left
                        tfResultsQueue.add(GOLD_MINERAL_POSITION.LEFT);
                        textToSpeech.speak("Gold Left ",
                                TextToSpeech.QUEUE_FLUSH,null);
                    } else {
                        // gold is center
                        tfResultsQueue.add(GOLD_MINERAL_POSITION.CENTER);
                        textToSpeech.speak("Gold Center ",
                                TextToSpeech.QUEUE_FLUSH,null);
                    }

                } else {
                    // We're seeing 2 silver minerals, do we care where they're at?
                    textToSpeech.speak("Gold Right ",
                            TextToSpeech.QUEUE_FLUSH,null);
                    tfResultsQueue.add(GOLD_MINERAL_POSITION.RIGHT);
                }
            } else if (mineralsFound == 3) {
                // Lucky case, code straight from the SDK example - although we really can't
                // see 3 minerals
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        tfResultsQueue.add(GOLD_MINERAL_POSITION.LEFT);
                        textToSpeech.speak("Gold Left ",
                                TextToSpeech.QUEUE_FLUSH,null);
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        textToSpeech.speak("Gold Right ",
                                TextToSpeech.QUEUE_FLUSH,null);
                        tfResultsQueue.add(GOLD_MINERAL_POSITION.RIGHT);
                    } else {
                        tfResultsQueue.add(GOLD_MINERAL_POSITION.CENTER);
                        textToSpeech.speak("Gold Center ",
                                TextToSpeech.QUEUE_FLUSH,null);
                    }
                }
            }
        } else {
            Log.d(LOG_TAG, "Mineral positions not detected");

            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    Log.d(LOG_TAG, "INC - TF recognized object labeled " + recognition.getLabel()
                            + " @ " + recognition.getLeft() + ", " + recognition.getBottom()
                            + "conf: " + recognition.getConfidence());
                }
            }
        }
    }

    private boolean isTimedOut() {
        // we have timeoutMillis
        // we have when the thread started (threadStartedTimeMillis)

        long nowMillis = System.currentTimeMillis();

        // how do we get from now to elapsed time?
        long elapsedTimeMillis =  nowMillis - threadStartedTimeMillis;

        // How do we compare elapsed time to millis to return true/false?

        if (elapsedTimeMillis >= timeoutMillis)  {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        // adjust confidence
        //tfodParameters.minimumConfidence = ...;

        // Replace above if you don't want the camera view to display on the robot controller
        //TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
