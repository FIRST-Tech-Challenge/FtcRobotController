/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.season1718;

import android.speech.tts.TextToSpeech;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;
import java.util.Queue;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

// TODO: Explain why we're using a thread instead of putting this directly in a state machine
// state

public class VuMarkThread extends Thread {
    // TODO: Explain what a queue is and why we're using it (might need to show caller's use of
    // it to do so!
    private final Queue<RelicRecoveryVuMark> vuMarkQueue;

    // These are straight from the season SDK demo OpMode
    private final VuforiaLocalizer vuforia;
    private final VuforiaTrackables relicTrackables;
    private final VuforiaTrackable relicTemplate;
    private final TextToSpeech textToSpeech;

    private final long timeoutMillis;

    private long threadStartedTimeMillis = 0;

    public VuMarkThread(Queue<RelicRecoveryVuMark> vuMarkQueue, HardwareMap hardwareMap,
                        long timeoutMillis) {
        this.vuMarkQueue = vuMarkQueue;
        this.timeoutMillis = timeoutMillis;

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AWG/LiP/////AAAAGX/9FIxTNUsrgFXqUcqtszoTTx7j2BRph1LwwuxLBRZq0OW2z5Zx5OBBvJljDpEaHS6qsAOwm0MxyLoPgik1HJt4UDW+x+KAkw/gH4sX3yJnQJrldAo1/r4Q/orVAoCO4oDRDgK9NGpBjIDG8zhx5wnFBIhal57MacPYrTkOEk/hVfPHVw/rYUaHTBDoiQ44E8s0hdDl2tjMzqmzBLsaFEhl0AbvR08SwY4nqmwNOSs0IIcUzLJEL62U3NQuB2lwFDZdta+8jOr+pwWBY+eBhoIRkFVAvza+2IvfusqFH5LXAvdExSHBEd6/DsDH3owa1FspJcRC16YGMHySETucHz4GtAycOa3xk1R7rrJAoBMR";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

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

    @Override
    public void run() {
        Log.d(LOG_TAG, "VuMark detection thread running");

        threadStartedTimeMillis = System.currentTimeMillis();

        Log.d(LOG_TAG, "Activating VuMark Trackables");

        relicTrackables.activate();

        Log.d(LOG_TAG, "Starting to look for VuMarks");

        while (!isTimedOut()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                vuMarkQueue.add(vuMark);
                Log.d(LOG_TAG, "Saw VuMark " + vuMark.name());

                if (textToSpeech != null) {
                    switch (vuMark) {
                        case LEFT:
                            textToSpeech.speak("Saw left.", TextToSpeech.QUEUE_FLUSH, null);
                            break;
                        case RIGHT:
                            textToSpeech.speak("Saw right",TextToSpeech.QUEUE_FLUSH,null);
                            break;
                        case CENTER:
                            textToSpeech.speak("Saw center", TextToSpeech.QUEUE_FLUSH,null);
                            break;
                        default:
                            // do nothing
                    }
                }

                break; //jump out of loop because we found something :D
            }
        }

        if (vuMarkQueue.isEmpty()) {
            Log.d(LOG_TAG, "VuMark not seen");
            textToSpeech.speak("Error four oh four vuMark not found ", TextToSpeech.QUEUE_FLUSH,null);
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
}
