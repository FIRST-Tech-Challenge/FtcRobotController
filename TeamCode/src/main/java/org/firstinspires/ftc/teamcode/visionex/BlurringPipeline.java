package org.firstinspires.ftc.teamcode.visionex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlurringPipeline extends OpenCvPipeline {
    private Telemetry telemetry;

    public BlurringPipeline(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat image) {

        Mat blurredImage = new Mat();
        Imgproc.GaussianBlur(image, blurredImage, new Size(15,15), 0, 0);

        telemetry.addLine("Image Blurred!");
        Imgcodecs.imwrite("sdcard/FIRST/output.png", blurredImage);
        telemetry.update();

        return image;
    }
}
