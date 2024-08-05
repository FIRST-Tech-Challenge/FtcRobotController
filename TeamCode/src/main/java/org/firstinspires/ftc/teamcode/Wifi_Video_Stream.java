package org.firstinspires.ftc.teamcode;  //place where the code is located


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
@Disabled

@TeleOp(name = "Wifi Video Stream", group = "Example")
public class Wifi_Video_Stream extends LinearOpMode {

    private static final int PORT = 25565;
    private static final int TIMEOUT = 5000;  // 5 seconds timeout
    private OpenCvCamera webcam;
    private FramePipeline pipeline;
    private volatile boolean sendData = true;
    private ServerSocket serverSocket;
    private Socket clientSocket;
    private DataOutputStream dos;
    private DataInputStream dis;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new FramePipeline();
        webcam.setPipeline(pipeline);

        // Use an anonymous class to implement AsyncCameraOpenListener
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT); // Set resolution manually
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error Code: " + errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        new Thread(new ServerRunnable()).start();

        while (opModeIsActive()) {
            idle();
        }

        // Clean up resources
        sendData = false;
        closeResources();
    }

    private void closeResources() {
        try {
            if (dis != null) dis.close();
            if (dos != null) dos.close();
            if (clientSocket != null && !clientSocket.isClosed()) clientSocket.close();
            if (serverSocket != null && !serverSocket.isClosed()) serverSocket.close();
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to close resources: " + e.getMessage());
            telemetry.update();
        }
    }

    private byte[] captureFrame() {
        Mat frame = new Mat();
        pipeline.getLatestFrame(frame); // Get the latest frame from the pipeline

        if (frame.empty()) {
            frame.release();
            return null; // No frame captured
        }

        // Convert the Mat to a byte array
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        MatOfByte matOfByte = new MatOfByte();
        Imgcodecs.imencode(".jpg", frame, matOfByte);
        byte[] byteArray = matOfByte.toArray();

        // Release resources
        frame.release();
        matOfByte.release();

        return byteArray;
    }

    // Define a custom OpenCvPipeline to process the frames
    public static class FramePipeline extends OpenCvPipeline {
        private Mat latestFrame = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            synchronized (this) {
                if (!latestFrame.empty()) {
                    latestFrame.release(); // Release previous frame
                }
                latestFrame = input.clone(); // Save the current frame
            }
            return input;
        }

        public synchronized void getLatestFrame(Mat frame) {
            latestFrame.copyTo(frame);
        }
    }

    private class ServerRunnable implements Runnable {
        @Override
        public void run() {
            try {
                serverSocket = new ServerSocket(PORT);
                serverSocket.setSoTimeout(TIMEOUT);
                telemetry.addData("Status", "Server started. Waiting for client connection...");
                telemetry.update();

                while (sendData) {
                    try {
                        clientSocket = serverSocket.accept();
                        clientSocket.setSoTimeout(TIMEOUT);
                        dos = new DataOutputStream(clientSocket.getOutputStream());
                        dis = new DataInputStream(clientSocket.getInputStream());

                        telemetry.addData("Status", "Client connected");
                        telemetry.update();

                        while (sendData && clientSocket.isConnected()) {
                            if (dis.available() > 0) {
                                String response = dis.readUTF();
                                while (response.equals("pildi protsessimine")) {
                                    telemetry.addData("Info", "Infot pole");
                                    telemetry.update();
                                }
                                telemetry.addData("Info", "info on");
                                telemetry.update();
                            }

                            byte[] imageData = captureFrame();
                            if (imageData != null) {
                                dos.writeInt(imageData.length);  // Send the length of the image data
                                dos.write(imageData);            // Send the image data itself
                                dos.flush();
                            }

                            Thread.yield();  // Yield to avoid blocking
                        }

                        telemetry.addData("Status", "Client disconnected");
                        telemetry.update();
                    } catch (SocketTimeoutException e) {
                        telemetry.addData("Error", "Client connection timeout");
                        telemetry.update();
                    } catch (IOException e) {
                        telemetry.addData("Error", e.getMessage());
                        telemetry.update();
                        e.printStackTrace();
                        try {
                            Thread.sleep(1000);  // Wait before retrying connection
                        } catch (Exception eew){

                        }
                    } finally {
                        if (dos != null) {
                            try {
                                dos.close();
                            } catch (IOException e) {
                                telemetry.addData("Error", "Failed to close DataOutputStream");
                                telemetry.update();
                            }
                        }
                        if (clientSocket != null && !clientSocket.isClosed()) {
                            try {
                                clientSocket.close();
                            } catch (IOException e) {
                                telemetry.addData("Error", "Failed to close client socket");
                                telemetry.update();
                            }
                        }
                    }
                }
                if (dos != null) {
                    dos.writeInt(-1); // Special flag to indicate the end of streaming
                    dos.flush();
                }
            } catch (IOException e) {
                telemetry.addData("Server Error", e.getMessage());
                telemetry.update();
                e.printStackTrace();
            } finally {
                if (serverSocket != null && !serverSocket.isClosed()) {
                    try {
                        serverSocket.close();
                    } catch (IOException e) {
                        telemetry.addData("Error", "Failed to close server socket");
                        telemetry.update();
                    }
                }
            }
        }
    }
}
