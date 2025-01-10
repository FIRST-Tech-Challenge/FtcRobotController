package org.firstinspires.ftc.teamcode.visionex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ResizingPipeline extends OpenCvPipeline {
    private Telemetry telemetry;

    public ResizingPipeline(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat image) {
        Imgproc.resize(image, image, new Size(64, 40));

        telemetry.addLine("Image Resized");
        telemetry.update();

        Imgproc.resize(image, image, new Size(64, 40));
        Imgcodecs.imwrite("sdcard/FIRST/output.png", image);
        return image;
    }
}
