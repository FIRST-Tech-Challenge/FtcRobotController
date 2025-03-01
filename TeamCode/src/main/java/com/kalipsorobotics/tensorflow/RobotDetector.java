package com.kalipsorobotics.tensorflow;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;

public class RobotDetector {
    private OpenCvCamera webcam;
    private Interpreter tfliteInterpreter;
    private final Telemetry telemetry;

    private static final String MODEL_NAME = "robotv2_model.tflite";
    private static final int IMAGE_SIZE = 224; // TensorFlow Lite model input size

    public RobotDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void initializeCamera(String webcamName, Object hardwareMap) {
        int cameraMonitorViewId = ((HardwareMap) hardwareMap)
                .appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", ((HardwareMap) hardwareMap).appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(((HardwareMap) hardwareMap).get(WebcamName.class, webcamName), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera", "Opened successfully");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Failed to open: " + errorCode);
                telemetry.update();
            }
        });
    }

    public void loadModel(Object hardwareMap) throws IOException {
        FileInputStream inputStream = new FileInputStream(
                ((com.qualcomm.robotcore.hardware.HardwareMap) hardwareMap)
                        .appContext.getAssets()
                        .openFd(MODEL_NAME)
                        .getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = ((com.qualcomm.robotcore.hardware.HardwareMap) hardwareMap).appContext.getAssets()
                .openFd(MODEL_NAME).getStartOffset();
        long declaredLength = ((com.qualcomm.robotcore.hardware.HardwareMap) hardwareMap).appContext.getAssets()
                .openFd(MODEL_NAME).getDeclaredLength();
        ByteBuffer modelBuffer = fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
        tfliteInterpreter = new Interpreter(modelBuffer);
    }

    public boolean isRobotDetected() {
        Bitmap bitmap = captureFrame();
        if (bitmap == null) {
            telemetry.addData("Error", "Failed to capture frame");
            telemetry.update();
            return false;
        }

        float[][] output = runModel(bitmap);
        return output[0][0] > 0.5; // Assuming output[0][0] is the probability of detecting a robot
    }

    private Bitmap captureFrame() {
        Mat frame = new Mat(); // Get the most recent frame from the camera

        if (frame.empty()) {
            return null;
        }

        Bitmap bitmap = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(frame, bitmap);
        return bitmap;
        //return Bitmap.createScaledBitmap(bitmap, IMAGE_SIZE, IMAGE_SIZE, true);
    }

    private ByteBuffer preprocessImage(Bitmap bitmap) {
        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(4 * IMAGE_SIZE * IMAGE_SIZE * 3);
        byteBuffer.order(ByteOrder.nativeOrder());

        int[] pixels = new int[IMAGE_SIZE * IMAGE_SIZE];
        bitmap.getPixels(pixels, 0, IMAGE_SIZE, 0, 0, IMAGE_SIZE, IMAGE_SIZE);

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
        ByteBuffer inputBuffer = preprocessImage(bitmap);
        float[][] output = new float[1][1]; // Assuming the model outputs a single probability
        tfliteInterpreter.run(inputBuffer, output);
        return output;
    }

    public void stopCamera() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }
}
