package com.kalipsorobotics.tensorflow;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import android.graphics.Bitmap;

import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;

@TeleOp(name = "Robot Detector", group = "TensorFlow")
public class MainActivity extends LinearOpMode {

    private OpenCvCamera webcam;
    private Interpreter tfliteInterpreter;
    private static final String MODEL_NAME = "robot_detector.tflite";

    @Override
    public void runOpMode() {
        // Initialize TensorFlow Lite model
        try {
            tfliteInterpreter = new Interpreter(loadModelFile(MODEL_NAME));
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to load model");
            telemetry.update();
            return;
        }

        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up the webcam and start the stream
        webcam.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                Bitmap bitmap = convertMatToBitmap(input);

                // Preprocess the image and run TensorFlow Lite
                float[][] output = runModel(bitmap);

                // Display results on telemetry
                if (output[0][0] > 0.5) {
                    telemetry.addData("Detection", "Robot Detected!");
                } else {
                    telemetry.addData("Detection", "No Robot Detected");
                }
                telemetry.update();

                return input; // Return the original frame for visualization
            }
        });

        //webcam.openCameraDeviceAsync(() -> webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT));

        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Loop while the OpMode is running
        }

        // Stop the camera
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    private ByteBuffer preprocessImage(Bitmap bitmap) {
        // Resize and normalize the image
        Bitmap resizedBitmap = Bitmap.createScaledBitmap(bitmap, 224, 224, true);
        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(4 * 224 * 224 * 3);
        byteBuffer.order(ByteOrder.nativeOrder());

        int[] pixels = new int[224 * 224];
        resizedBitmap.getPixels(pixels, 0, 224, 0, 0, 224, 224);

        for (int pixel : pixels) {
            int r = (pixel >> 16) & 0xFF;
            int g = (pixel >> 8) & 0xFF;
            int b = pixel & 0xFF;

            byteBuffer.putFloat(r / 255.0f);
            byteBuffer.putFloat(g / 255.0f);
            byteBuffer.putFloat(b / 255.0f);
        }

        return byteBuffer;
    }

    private float[][] runModel(Bitmap bitmap) {
        // Prepare input buffer
        ByteBuffer inputBuffer = preprocessImage(bitmap);

        // Define output buffer
        float[][] output = new float[1][1]; // Assuming the model outputs one probability value

        // Run inference
        tfliteInterpreter.run(inputBuffer, output);

        return output;
    }

    private ByteBuffer loadModelFile(String modelPath) throws IOException {
        FileInputStream inputStream = new FileInputStream(hardwareMap.appContext.getAssets().openFd(modelPath).getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = hardwareMap.appContext.getAssets().openFd(modelPath).getStartOffset();
        long declaredLength = hardwareMap.appContext.getAssets().openFd(modelPath).getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    private Bitmap convertMatToBitmap(Mat input) {
        Bitmap bitmap = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(input, bitmap);
        return bitmap;
    }
}
