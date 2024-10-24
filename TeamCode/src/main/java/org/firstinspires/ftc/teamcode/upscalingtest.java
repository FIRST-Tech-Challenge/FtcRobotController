package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;

import java.net.Socket;
import java.io.OutputStream;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.io.ByteArrayOutputStream;
import java.net.URL;
import java.net.HttpURLConnection;

// an FTC TeleOp OpMode that retrieves video frames from Limelight, sends them to an external device for AI upscaling,
// and receives the upscaled image to continue using Limelight's vision capabilities.

@TeleOp(name = "Limelight AI Upscaling", group = "Macro Tests")
@Disabled
public class upscalingtest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);  // switch to the default pipeline.
        limelight.start();  // start capturing video frames.

        telemetry.addData("Status", "Ready. Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // retrieve the latest frame from Limelight
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // function to convert Limelight result to an OpenCV Mat (OpenCV image)
                Mat imageFrame = retrieveLimelightFrame(result);

                // send to AI processor for upscaling
                Mat upscaledFrame = upscaleImage(imageFrame);

                // process the upscaled frame (i.e., send back to Limelight for further use)
                processUpscaledFrame(upscaledFrame);

                telemetry.addData("Image", "Frame processed and upscaled.");
                telemetry.update();
            }
        }

        limelight.stop();
    }

    /**
     * Retrieve the frame from the Limelight's result and convert it to an OpenCV Mat object.
     */
    private Mat retrieveLimelightFrame(LLResult result) {
        // fetch the raw image data from Limelight
        byte[] imageData = fetchImageFromLimelight();  // correct method to retrieve raw image data.
        // convert the byte array to Mat using MatOfByte
        MatOfByte matOfByte = new MatOfByte(imageData);
        Mat imageFrame = Imgcodecs.imdecode(matOfByte, Imgcodecs.IMREAD_COLOR);  // decode into Mat format.

        return imageFrame;
    }

    /**
     * Function designed to fetch image data from Limelight.
     */
    private byte[] fetchImageFromLimelight() {
        byte[] imageData = null;
        try {
            // connect to Limelight's video stream (using Limelight's HTTP URL for the raw JPEG stream)
            URL url = new URL("http://10.xx.xx.xx:5800/stream.mjpg");  // replace 10.xx.xx.xx with the actual IP address assigned to your Limelight on the network.
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            InputStream inputStream = connection.getInputStream();

            // read the image data from the connection stream
            ByteArrayOutputStream buffer = new ByteArrayOutputStream();
            int nRead;
            byte[] data = new byte[16384];
            while ((nRead = inputStream.read(data, 0, data.length)) != -1) {
                buffer.write(data, 0, nRead);
            }
            buffer.flush();
            imageData = buffer.toByteArray();

            inputStream.close();
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to fetch image from Limelight: " + e.getMessage());
        }
        return imageData;
    }

    /**
     * Send the image frame to an external AI processor for upscaling.
     */
    private Mat upscaleImage(Mat imageFrame) {
        try {
            // connect to the AI processor (assume it's running on a local network or connected via IP).
            Socket socket = new Socket("192.168.x.x", 12345);  // replace with AI processor IP address and port.

            // serialize the image frame.
            byte[] frameData = matToByteArray(imageFrame);

            // send the image frame over the network.
            OutputStream outputStream = socket.getOutputStream();
            outputStream.write(frameData);
            outputStream.flush();

            // receive the upscaled frame back from the AI processor.
            InputStream inputStream = socket.getInputStream();
            ByteArrayOutputStream buffer = new ByteArrayOutputStream();
            int nRead;
            byte[] data = new byte[16384];
            while ((nRead = inputStream.read(data, 0, data.length)) != -1) {
                buffer.write(data, 0, nRead);
                buffer.write(data, 0, nRead);
            }
            buffer.flush();
            byte[] upscaledData = buffer.toByteArray();
            // convert the received data back into a Mat object.
            MatOfByte matOfByte = new MatOfByte(upscaledData);
            Mat upscaledFrame = Imgcodecs.imdecode(matOfByte, Imgcodecs.IMREAD_COLOR);

            // close the connection
            socket.close();
            return upscaledFrame;

        } catch (Exception e) {
            telemetry.addData("Error", "Unable to upscale image: " + e.getMessage());
            // return original frame if something fails
            return imageFrame;
        }
    }

    /**
     * Process the upscaled frame and send it back to the Limelight or use it in some other part of the vision pipeline.
     */
    private void processUpscaledFrame(Mat upscaledFrame) {
        // convert the upscaled frame back into a format usable by Limelight or FTC system (if needed).
        byte[] processedData = matToByteArray(upscaledFrame);

        // use processedData in your vision processing or display telemetry.
        telemetry.addData("Upscaled Image", "Image processed and returned to Limelight.");
    }

    /**
     * Helper method to convert OpenCV Mat object into a byte array.
     */
    private byte[] matToByteArray(Mat mat) {
        ByteBuffer buffer = ByteBuffer.allocate((int) (mat.total() * mat.elemSize()));
        mat.get(0, 0, buffer.array());
        return buffer.array();
    }
}


/** disregard everything below, dont delete rn **/
/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.net.Socket;
import java.io.OutputStream;
import java.io.InputStream;
import java.nio.ByteBuffer;

import java.net.URL;
import java.net.HttpURLConnection;

// an FTC TeleOp OpMode that retrieves video frames from Limelight, sends them to an external device for AI upscaling,
// and receives the upscaled image to continue using Limelight's vision capabilities.

@TeleOp(name = "Limelight AI Upscaling", group = "Macro Tests")
@Disabled
public class upscalingtest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);  // Switch to the default pipeline.
        limelight.start();  // Start capturing video frames.

        telemetry.addData("Status", "Ready. Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // retrieve the latest frame from Limelight
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // function to convert Limelight result to a Mat (OpenCV image)
                Mat imageFrame = retrieveLimelightFrame(result);

                // send to AI processor for upscaling
                Mat upscaledFrame = upscaleImage(imageFrame);

                // process the upscaled frame (i.e., send back to Limelight for further use)
                processUpscaledFrame(upscaledFrame);

                telemetry.addData("Image", "Frame processed and upscaled.");
                telemetry.update();
            }
        }

        limelight.stop();
    }


     // retrieve the frame from the Limelight's result and convert it to an OpenCV Mat object.

    private Mat retrieveLimelightFrame(LLResult result) {
        // assuming result contains the raw image frame in some format
        // convert the image frame to an OpenCV Mat object.
        //byte[] imageData = result.getRawImageData();  // Pseudocode, need actual Limelight API support.
        //Mat imageFrame = Imgcodecs.imdecode(new Mat(imageData), Imgcodecs.IMREAD_COLOR);  // Decode into Mat format.
        return null;//imageFrame;
    }


     // send the image frame to an external AI processor for upscaling.

    private Mat upscaleImage(Mat imageFrame) {
        try {
            // connect to the AI processor (assume it's running on a local network or connected via IP).
            Socket socket = new Socket("192.168.x.x", 12345);  // Replace with AI processor IP address and port.

            // serialize the image frame.
            byte[] frameData = matToByteArray(imageFrame);

            // send the image frame over the network.
            OutputStream outputStream = socket.getOutputStream();
            outputStream.write(frameData);
            outputStream.flush();

            // receive the upscaled frame back from the AI processor.
            InputStream inputStream = socket.getInputStream();
            //byte[] upscaledData = new byte[frameData.length * 4];  // Assuming the upscaled image is larger.
           // inputStream.read(upscaledData);

            // convert the received data back into a Mat object.
            //Mat upscaledFrame = Imgcodecs.imdecode(new Mat(upscaledData), Imgcodecs.IMREAD_COLOR);
            // close the connection
            socket.close();
            return null;//upscaledFrame;

        } catch (Exception e) {
            telemetry.addData("Error", "Unable to upscale image: " + e.getMessage());
            // return original frame if something fails
            return imageFrame;
        }
    }


     // process the upscaled frame and send it back to the Limelight or use it in some other part of the vision pipeline.

    private void processUpscaledFrame(Mat upscaledFrame) {
        // convert the upscaled frame back into a format usable by Limelight or FTC system (if needed).
        byte[] processedData = matToByteArray(upscaledFrame);

        // use processedData in your vision processing or display telemetry.
        telemetry.addData("Upscaled Image", "Image processed and returned to Limelight.");
    }


     // helper method to convert OpenCV Mat object into a byte array.

    private byte[] matToByteArray(Mat mat) {
        ByteBuffer buffer = ByteBuffer.allocate((int) (mat.total() * mat.elemSize()));
        mat.get(0, 0, buffer.array());
        return buffer.array();
    }
}*/