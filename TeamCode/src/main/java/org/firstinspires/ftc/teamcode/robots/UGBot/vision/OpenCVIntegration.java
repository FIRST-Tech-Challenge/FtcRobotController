package org.firstinspires.ftc.teamcode.robots.UGBot.vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.BlobStats;
import org.firstinspires.ftc.teamcode.util.VisionUtils;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.BlockingQueue;

public class OpenCVIntegration implements VisionProvider {
    public OpenCvCamera camera;
    private boolean enableDashboard;
    public Pipeline pipeline;


    @Override
    public void initializeVision(HardwareMap hardwareMap, Viewpoint viewpoint, boolean enableDashboard) {
        pipeline = new Pipeline(enableDashboard);
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(() -> {
                camera.startStreaming(Constants.WEBCAM_WIDTH, Constants.WEBCAM_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
        });
    }

    @Override
    public void shutdownVision() {
        camera.closeCameraDevice();
        camera.stopStreaming();
    }
    
    @Override
    public StackHeight detect() {
        return pipeline.getLastStackHeight();
    }

    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public Mat process(Mat source0) {
        return null;
    }

    @Override
    public void reset() {}
}
