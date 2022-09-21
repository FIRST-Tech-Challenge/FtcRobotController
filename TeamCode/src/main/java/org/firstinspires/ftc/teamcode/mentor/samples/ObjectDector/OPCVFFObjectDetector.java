package org.firstinspires.ftc.teamcode.mentor.samples.ObjectDector;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class OPCVFFObjectDetector extends OpenCvPipeline {

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


    /**
     *
     * @param width The width of the image (check your camera)
     */
    public OPCVFFObjectDetector(int width, int height,Telemetry telemetry) {
        this.width = width;
        this.height = height;
        this.telemetry = telemetry;

        cvDNN = new Dnn();
        net = cvDNN.readNetFromTensorflow("/sdcard/FIRST/opencv/models/freight_frenzy_optimized_graph.pb");

    }


    @Override
    public Mat processFrame(Mat inputImage) {

        telemetry.addLine("Inside ProcessFrame");
        telemetry.update();

        Mat blob = null;
        Mat detections = null;
        List<DNNObject> objectList = new ArrayList<>();

        int cols = inputImage.cols();
        int rows = inputImage.rows();

        Mat imageRGB = new Mat();
        Imgproc.cvtColor(inputImage,imageRGB,Imgproc.COLOR_RGBA2RGB);

        blob = Dnn.blobFromImage(imageRGB, inScaleFactor,
                new Size(width, height),
                new Scalar(meanVal, meanVal, meanVal),
                false, false);

        net.setInput(blob);
        detections = net.forward();

        if(detections != null){
            //telemetry.addData("got detections",detections.total());
            //telemetry.update();

            //detections = detections.reshape(1, (int) detections.total() / 7);


            //all detected objects
            for (int i = 0; i < detections.rows(); ++i) {

                //telemetry.addData("got rows",detections.rows());
                //telemetry.update();

                double confidence = detections.get(i, 2)[0];

                if (confidence < thresholdDnn)
                    continue;

                telemetry.addLine("confident");
                telemetry.update();

                int classId = (int) detections.get(i, 1)[0];

                //calculate position
                int xLeftBottom = (int) (detections.get(i, 3)[0] * cols);
                int yLeftBottom = (int) (detections.get(i, 4)[0] * rows);
                Point leftPosition = new Point(xLeftBottom, yLeftBottom);

                int xRightTop = (int) (detections.get(i, 5)[0] * cols);
                int yRightTop = (int) (detections.get(i, 6)[0] * rows);
                Point rightPosition = new Point(xRightTop, yRightTop);

                /*float centerX = (xLeftBottom + xRightTop) / 2;
                float centerY = (yLeftBottom - yRightTop) / 2;
                Point centerPoint = new Point(centerX, centerY);


                /*DNNObject dnnObject = new DNNObject(classId, classNames[classId].toString(), leftPosition, rightPosition, centerPoint);
                objectList.add(dnnObject);

                Imgproc.rectangle(inputImage, dnnObject.getLeftBottom(), dnnObject.getRightTop(), new Scalar(255, 0, 0), 1);*/
            }
        }

        return imageRGB;
    }

}