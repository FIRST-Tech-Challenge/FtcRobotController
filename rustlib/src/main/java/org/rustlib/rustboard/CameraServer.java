package org.rustlib.rustboard;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;

import java.util.ArrayList;
import java.util.List;


public class CameraServer implements VisionProcessor {
    private static final List<CameraServer> instances = new ArrayList<>();
    private MatOfByte binaryImg = new MatOfByte();

    public CameraServer(CameraName camera, int port) {
        new VisionPortal.Builder().setCamera(camera).addProcessor(this).build();

//        Server server = new Server(port);
//        server.setHandler(new Handler.Abstract() {
//            @Override
//            public boolean handle(Request request, NanoHTTPD.Response response, Callback callback) throws Exception {
//                byte[] imageBytes = binaryImg.toArray();
//                response.write(true, ByteBuffer.allocate(imageBytes.length).put(imageBytes), callback);
//                callback.succeeded();
//                return true;
//            }
//        });
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {
        binaryImg = new MatOfByte(frame);
        return frame;
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
