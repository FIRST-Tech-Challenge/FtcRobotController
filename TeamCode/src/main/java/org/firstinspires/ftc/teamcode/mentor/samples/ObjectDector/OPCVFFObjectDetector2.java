package org.firstinspires.ftc.teamcode.mentor.samples.ObjectDector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class OPCVFFObjectDetector2 extends OpenCvPipeline {

    private int width; // width of the image
    private int height = 240;
    private double inScaleFactor = 0.007843;
    private double thresholdDnn =  0.2;
    private double meanVal = 127.5;
    private Dnn cvDNN = null;
    private Net net = null;
    private Telemetry telemetry = null;

    private final String[] classNames = {"Background",
            "Plane", "Ship", "Side Truck", "Front Truck"};

    private static List<Scalar> colors=new ArrayList<>();

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public OPCVFFObjectDetector2(int width, int height, Telemetry telemetry) {
        this.width = width;
        this.height = height;
        this.telemetry = telemetry;

        cvDNN = new Dnn();
        net = cvDNN.readNetFromTensorflow("/sdcard/FIRST/opencv/models/freight_frenzy_optimized_graph.pb");
        for(int i=0; i<classNames.length; i++)
            colors.add(randomColor());

    }


    @Override
    public Mat processFrame(Mat inputFrame) {



        Mat frame = new Mat();
        Imgproc.cvtColor(inputFrame, frame, Imgproc.COLOR_RGBA2RGB);
        Size frame_size = new Size(width, height);
        Scalar mean = new Scalar(meanVal);

        Mat blob = Dnn.blobFromImage(frame, inScaleFactor,
                new Size(width, height),
                new Scalar(meanVal, meanVal, meanVal),
                false, false);
        Mat imageBlob = Dnn.blobFromImage(frame, 0.00392, new Size(width,height),new Scalar(0, 0, 0),/*swapRB*/false, /*crop*/false);
        net.setInput(imageBlob);

        List<Mat> result = new ArrayList<>();
        List<String> outBlobNames = net.getUnconnectedOutLayersNames();

        net.forward(result,outBlobNames);
        float confThreshold = 0.5f;

        for (int i = 0; i < result.size(); ++i) {
            // each row is a candidate detection, the 1st 4 numbers are
            // [center_x, center_y, width, height], followed by (N-4) class probabilities
            Mat level = result.get(i);
            for (int j = 0; j < level.rows(); ++j) {
                Mat row = level.row(j);
                Mat scores = row.colRange(5, level.cols());
                Core.MinMaxLocResult mm = Core.minMaxLoc(scores);
                float confidence = (float) mm.maxVal;
                Point classIdPoint = mm.maxLoc;

                telemetry.addData("Confidence", confidence);
                telemetry.update();

                if (confidence > confThreshold) {



                    int centerX = (int) (row.get(0, 0)[0] * frame.cols());
                    int centerY = (int) (row.get(0, 1)[0] * frame.rows());
                    int width = (int) (row.get(0, 2)[0] * frame.cols());
                    int height = (int) (row.get(0, 3)[0] * frame.rows());

                    int left = (int) (centerX - width * 0.5);
                    int top =(int)(centerY - height * 0.5);
                    int right =(int)(centerX + width * 0.5);
                    int bottom =(int)(centerY + height * 0.5);

                    Point left_top = new Point(left, top);
                    Point right_bottom=new Point(right, bottom);
                    Point label_left_top = new Point(left, top-5);
                    DecimalFormat df = new DecimalFormat("#.##");

                    int class_id = (int) classIdPoint.x;
                    String label= classNames[3].toString() + ": " + df.format(confidence);
                    Scalar color= colors.get(3);

                    Imgproc.rectangle(frame, left_top,right_bottom , color, 3, 2);
                    Imgproc.putText(frame, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 0, 0), 4);
                    Imgproc.putText(frame, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255), 2);
                }
            }
        }
        return frame;
    }

    private Scalar randomColor() {
        Random random = new Random();
        int r = random.nextInt(255);
        int g = random.nextInt(255);
        int b = random.nextInt(255);
        return new Scalar(r,g,b);
    }

}