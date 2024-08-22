package org.rustlib.rustboard;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.rustlib.vision.FrameProcessor;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;

import fi.iki.elonen.NanoHTTPD;

public class CameraServer extends NanoHTTPD implements FrameProcessor {
    private static int nextAvailablePort = 10000;
    private final int port;
    private VisionPortal visionPortal;
    private VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
    private final CameraName camera;
    protected byte[] imgBytes = encode(new Mat());

    public CameraServer(HardwareMap hardwareMap, String cameraName, int port) {
        super(port);
        this.port = port;
        camera = hardwareMap.get(CameraName.class, cameraName);
        visionPortalBuilder.setCamera(camera).addProcessor(this);
    }

    public CameraServer(HardwareMap hardwareMap, String cameraName) {
        this(hardwareMap, cameraName, nextAvailablePort);
        nextAvailablePort += 1;
    }

    public int getPort() {
        return port;
    }

    private byte[] encode(Mat mat) {
        if (mat.empty()) {
            return new byte[]{};
        } else {
            MatOfByte matOfByte = new MatOfByte();
            Imgcodecs.imencode(".png", mat, matOfByte);
            return matOfByte.toArray();
        }
    }

    @Override
    public Response serve(IHTTPSession session) {
        return newFixedLengthResponse(Response.Status.OK, "image/png", new ByteArrayInputStream(imgBytes), imgBytes.length);
    }


    public InetSocketAddress getServerAddress() throws UnknownHostException {
        return new InetSocketAddress(InetAddress.getLocalHost(), 8080);
    }

    @Override
    public final void start() throws IOException {
        visionPortal = visionPortalBuilder.build();
        super.start();
    }

    @Override
    public final void stop() {
        try {
            visionPortal.stopStreaming();
        } finally {
            super.stop();
        }
    }

    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {
        imgBytes = encode(frame);
        return frame;
    }
}
