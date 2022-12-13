package org.firstinspires.ftc.teamcode.pipelines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodekt.util.MU;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Class to detect the pole in an EasyOpenCV pipeline.
 */
public class PoleDetector extends OpenCvPipeline {
    /**
     * Telemetry object to display data to the console
     */
    private Telemetry telemetry;

    private DetectedCircle poleLocationPixels;

    /**
     * Constructor to assign the telemetry object and actually have telemetry work.
     * This works because the OpenCvPipeline object has this built in interface to
     * use telemetry.
     * @param telemetry the input Telemetry type object.
     */
    public PoleDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
        poleLocationPixels = new DetectedCircle();
    }


    /**
     * Apply a canny edge detector to an input matrix.
     * @param src is the source matrix - what is put into the function
     * @return a mat with only the edges of the main bodies given some contrast threshold.
     */
    public Mat canny(Mat src){
        // Define the size of the input image
        int frameWidth = src.cols();
        int frameHeight = src.rows();

        Mat gray = new Mat(frameHeight, frameWidth, src.type());
        Mat edges = new Mat(frameHeight, frameWidth, src.type());

        Imgproc.cvtColor(src, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.blur(gray, gray, new Size(3, 3));
        Imgproc.Canny(gray, edges, 1, 150);

        Mat circles = new Mat();
        Imgproc.HoughCircles(edges, circles, Imgproc.HOUGH_GRADIENT, 1.5, frameWidth/4.0, 250, 75, 20, 90);

        for(int x=0; x<circles.cols(); x++){
            double[] c = circles.get(0, x);
            Point center = new Point(Math.round(c[0]), Math.round(c[1]));

            Imgproc.circle(src, center, 1, new Scalar(0, 200, 0), 8, 2, 0);
            int radius = (int)Math.round(c[2]);
            Imgproc.circle(src, center, radius, new Scalar(0, 200, 0), 8, 2, 0);
            // Capture the pixel point position as a variable to access
            if(poleLocationPixels.x == 0 && poleLocationPixels.y == 0) {
                poleLocationPixels.x = center.x;
                poleLocationPixels.y = center.y;
            }
            else {
                poleLocationPixels.x = MU.avg(poleLocationPixels.x, center.x);
                poleLocationPixels.y = MU.avg(poleLocationPixels.y, center.y);
            }
            poleLocationPixels.radius = radius;
        }


        gray.release();
        edges.release();

        return src;
    }

    public DetectedCircle getPoleLocationPixels(){
        return poleLocationPixels;
    }

    public DetectedCircle getPoleLocationRelative(){
        // TODO: Apply some operation to convert pixel location to x/y in cm!
        return poleLocationPixels;
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