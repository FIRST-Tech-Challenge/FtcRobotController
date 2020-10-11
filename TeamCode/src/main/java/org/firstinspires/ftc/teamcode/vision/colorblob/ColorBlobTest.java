/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.vision.colorblob;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.util.VisionUtils;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.teamcode.util.VisionUtils.getColumnPos;
import static org.firstinspires.ftc.teamcode.util.VisionUtils.getImageFromFrame;

@Autonomous(name="ColorBlobTest", group ="Concept")
//@Disabled
public class ColorBlobTest extends LinearOpMode {

    public static final String TAG = "Blob Test";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(1);


        VuforiaTrackables beaconTargets = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        beaconTargets.get(0).setName("RelicTemplate");

        VuforiaTrackable relicCodex  = beaconTargets.get(0);
        relicCodex.setName("CryptKey");  // Cryptokey


        Vuforia.setHint (HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);
        ColorBlobDetector mDetector = new ColorBlobDetector();
        boolean visionConfigured = false;


                /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.addData("OpenCV", Core.NATIVE_LIBRARY_NAME);
        telemetry.update();


        waitForStart();



        /** Start tracking the data sets we care about. */
        beaconTargets.activate();


        while (opModeIsActive()) {

            Image rgb = null;
            Mat img = null;
            int jewelConfig = 20;
            double colPos = 0;

            VuforiaLocalizer.CloseableFrame frame = null;


            if(visionConfigured)
            {
                try {
                    frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
                    colPos = getColumnPos(getImageFromFrame(frame, PIXEL_FORMAT.RGB565),1, mDetector);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            else //setup colorblobtracker
            {
                mDetector.setHsvColor(VisionUtils.OTHER_RED_HIGH);
                visionConfigured=true;
            }
            try {

                jewelConfig = VisionUtils.getJewelConfig(getImageFromFrame(frame, PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) relicCodex.getListener(), vuforia.getCameraCalibration());

                if (frame != null) {
                    long numImages = frame.getNumImages();
                    Log.d(TAG, String.valueOf(numImages));

                    for (int i = 0; i < numImages; i++) {
                        Log.d(TAG, String.valueOf(i));
                        if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                            rgb = frame.getImage(i);
                            break;
                        }
                    }

                    if (rgb != null) {
                        Bitmap bmp = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                        bmp.copyPixelsFromBuffer(rgb.getPixels());

                        img = new Mat();
                        Utils.bitmapToMat(bmp, img);
                        telemetry.addData("Img", "Converted");
                        telemetry.update();
                    }

                    frame.close(); //doesn't seem to matter if this is present or not
                }
            }
                catch (Exception e) {

                    Log.d(TAG, e.toString());

                }//if frame null
            telemetry.addData("jewelConfig", jewelConfig);
            telemetry.addData("columnPos", colPos);
            telemetry.update();

            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            telemetry.update();


        }//while opmodeactive
    }//runopmode

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
