
package com.kalipsorobotics.tensorflow;

import androidx.annotation.CallSuper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;

import java.nio.ByteBuffer;

public class TensorflowDetection extends LinearOpMode {
        private Interpreter tflite;

        @Override
        public void runOpMode() throws InterruptedException {
            // Load the model file from the assets folder
            try {
                // Load model from assets directory
                //FileUtil.loadmappedFile = method to read the model from the assets folder, and loading into the thing called 'ByteBuffer'
                //We use bytebuffer because TFlite model is loaded into mem as a bytebuffer thing
                //hardwaremap to access context and laod model
                ByteBuffer modelFile = FileUtil.loadMappedFile(hardwareMap.appContext, "robotv2_model.tflite");
                tflite = new Interpreter(modelFile);
            } catch (Exception e) {
                telemetry.addData("Error", "Failed to load TensorFlow Lite model");
                telemetry.update();
                return;
            }

            waitForStart();

            try {
                while (opModeIsActive()) {
                    float[] input = new float[224 * 224 * 3];
                    float[] output = new float[224 * 224 * 3];

                    //runs inference and places result in output array (below)
                    tflite.run(input, output);

                    telemetry.addData("Output", output[0]);
                    telemetry.update();
                }
            } finally {
                myStop();
            }

            // Continue with your OpMode code...
        }

      private void myStop() {
            // Don't forget to close the interpreter when done
        if (tflite != null) {
            tflite.close();
            }

        }

}
