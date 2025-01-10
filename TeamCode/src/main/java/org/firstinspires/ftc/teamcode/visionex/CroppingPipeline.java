package org.firstinspires.ftc.teamcode.visionex;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CroppingPipeline extends OpenCvPipeline {

    private Telemetry telemetry;

    public CroppingPipeline(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat image) {
        image = image.submat(new Rect(0,0, 640, 240));

        telemetry.addLine("Image Cropped!");
        telemetry.update();

        Imgcodecs.imwrite("sdcard/FIRST/output.png", image);
        Imgproc.resize(image, image, new Size(640,640));
        return image;
    }
}
