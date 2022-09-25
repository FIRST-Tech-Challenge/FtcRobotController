package org.firstinspires.ftc.teamcode.Pipelines;

import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Class to apply a canny edge detector in an EasyOpenCV pipeline.
 */
public class CannyEdge extends OpenCvPipeline {
    /**
     * Telemetry object to display data to the console
     */
    private Telemetry telemetry = null;

    /**
     * Constructor to assign the telemetry object and actually have telemetry work.
     * This works because the OpenCvPipeline object has this built in interface to
     * use telemetry.
     * @param telemetry the input Telemetry type object.
     */
    public CannyEdge(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    /**
     * Apply a canny edge detector to an input matrix.
     * @param src is the source matrix - what is put into the function
     * @return a mat with only the edges of the main bodies given some contrast threshold.
     */
    public Mat canny(Mat src){
        Mat gray = new Mat(src.rows(), src.cols(), src.type());
        Mat edges = new Mat(src.rows(), src.cols(), src.type());
        Mat dst = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));

        Imgproc.cvtColor(src, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.blur(gray, edges, new Size(3, 3));
        Imgproc.Canny(edges, edges, 5, 100);
        src.copyTo(dst, edges);

        gray.release();
        edges.release();
        src.release();

        telemetry.addLine("canny(...) running!");
        telemetry.update();

        return dst;
    }

    /**
     * Process the frame by returning the canny edge of the input image
     * @param img the input image to this pipeline.
     * @return The canny edge version of the input image.
     */
    @Override
    public Mat processFrame(Mat img) {
        return canny(img);
    }
}