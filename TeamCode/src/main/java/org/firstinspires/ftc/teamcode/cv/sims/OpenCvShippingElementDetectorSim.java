package org.firstinspires.ftc.teamcode.cv.sims;

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
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class OpenCvShippingElementDetectorSim extends OpenCvPipeline {



    public enum TSELocation {
        DUCKSIDE_RED_LEVEL_3,
        DUCKSIDE_RED_LEVEL_2,
        DUCKSIDE_RED_LEVEL_1,
        WAREHOUSESIDE_RED_LEVEL_3,
        WAREHOUSESIDE_RED_LEVEL_2,
        WAREHOUSESIDE_RED_LEVEL_1,
        DUCKSIDE_BLUE_LEVEL_3,
        DUCKSIDE_BLUE_LEVEL_2,
        DUCKSIDE_BLUE_LEVEL_1,
        WAREHOUSESIDE_BLUE_LEVEL_3,
        WAREHOUSESIDE_BLUE_LEVEL_2,
        WAREHOUSESIDE_BLUE_LEVEL_1,
        NONE
    }

    TSELocation location;
    Map<TSELocation, Integer> levels = new HashMap<>();


    public int width = 224; // width of the image
    public int height = 240;

    public double inScaleFactor = 0.0279;
    private double thresholdDnn =  0.6;
    public double redMeanVal = 5;
    public double greenMeanVal = 168.6;
    public double blueMeanVal = 62.3;


    private Dnn cvDNN = null;
    private Net net = null;
    private Telemetry telemetry = null;
    private Mat imageRGB = new Mat();

    private Scalar GREEN = new Scalar(0, 0, 255);

    //used to store maximum confidence amoung samples taken.  Only overwrite the level if we get a higher confidence
    private float maxConfidence = 0.0f;
    private boolean absolutelySure = false;

    static final private float CONF_THRESHOLD = 0.65f;



    private final String[] classNamesDuckSideBlue = {"background",
            "duckside_blue_level_3", "duckside_blue_level_2", "duckside_blue_level_1" };


    private String[] classNames = classNamesDuckSideBlue;

    private final String modelPathDuckSideBlue = "/Users/alex/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_barcodes/duckside_blue/converted_keras_duckside_blue/optimized/freight_frenzy_barcodes_duckside_blue_graph.pb";

    private String modelPath = modelPathDuckSideBlue;

    private List<TSELocation> locationSamples = new ArrayList<>();

    private Mat blob = null;
    private Mat detections = null;
    private Mat row = null;


    //private static List<Scalar> colors=new ArrayList<>();


    public OpenCvShippingElementDetectorSim(Telemetry telemetry) {

        this.telemetry = telemetry;

        levels.put(TSELocation.NONE,0);

        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_1,1);


        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_2,2);


        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_3,3);


        location = TSELocation.NONE;


        cvDNN = new Dnn();
        net = cvDNN.readNetFromTensorflow(modelPath);


    }
    public OpenCvShippingElementDetectorSim(int width, int height, Telemetry telemetry) {
        this.width = width;
        this.height = height;
        this.telemetry = telemetry;

        levels.put(TSELocation.NONE,0);

        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_1,1);


        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_2,2);


        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_3,3);

        location = TSELocation.NONE;

        classNames = classNamesDuckSideBlue;
        modelPath = modelPathDuckSideBlue;

        cvDNN = new Dnn();
        net = cvDNN.readNetFromTensorflow(modelPath);

    }


    @Override
    public Mat processFrame(Mat inputFrame) {

        //telemetry.clear();

        Imgproc.cvtColor(inputFrame,imageRGB,Imgproc.COLOR_RGBA2RGB);

        blob = Dnn.blobFromImage(imageRGB, inScaleFactor,
                new Size(width, height),
                new Scalar(redMeanVal, greenMeanVal, blueMeanVal),
                false, true);

        net.setInput(blob);
        detections = net.forward();

        telemetry.addData("# 0f detects: ", detections.rows());
        telemetry.update();

        for (int i = 0; i < detections.rows(); ++i) {
            row = detections.row(i);

            Core.MinMaxLocResult mm = Core.minMaxLoc(row);

            float confidence = (float) mm.maxVal;
            Point classIdPoint = mm.maxLoc;

            telemetry.addData("confidence is this: ", confidence);
            telemetry.update();
            if (confidence >= CONF_THRESHOLD) {

                int centerX = (int) (row.get(0, 0)[0] * row.cols());
                int centerY = (int) (row.get(0, 1)[0] * row.rows());
                int width = (int) (row.get(0, 2)[0] * row.cols());
                int height = (int) (row.get(0, 3)[0] * row.rows());

                telemetry.addLine(String.format("%d %d %d %d,",centerX,centerY,width,height));

                int left = (int) (centerX - width * 0.5);
                int top = (int) (centerY - height * 0.5);


                Point label_left_top = new Point(left, top + 30);
                DecimalFormat df = new DecimalFormat("#.##");

                int class_id = (int) classIdPoint.x;
                String className = classNames[class_id].toString();
                String label =  className + ": " + df.format(confidence);

                //telemetry.addData("This is a real new", label);
                //telemetry.update();

                switch (className)
                {
                    case "duckside_blue_level_1":
                        location = TSELocation.DUCKSIDE_BLUE_LEVEL_1;
                        break;

                    case "duckside_blue_level_2":
                        location = TSELocation.DUCKSIDE_BLUE_LEVEL_2;
                        break;

                    case "duckside_blue_level_3":
                        location = TSELocation.DUCKSIDE_BLUE_LEVEL_3;
                        break;



                    default:
                        location = TSELocation.NONE;
                }

                telemetry.addData("The location being set is", location);
                telemetry.addData("With confidence", confidence);

                telemetry.addData("This is a real location", getLocation());
                telemetry.update();

                Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(0, 0, 0), 4);
                Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(255, 255, 255), 2);
            }

        }


        return imageRGB;
    }


    public TSELocation getLocation() {final Integer[] value = new Integer[1];

        return location;
    }
    public int getTSELevel(){
        //telemetry.addData("getTSELevel", location);
        return levels.get(location);
    }
}