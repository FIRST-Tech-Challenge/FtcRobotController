package org.firstinspires.ftc.teamcode.cv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mentor.samples.ObjectDector.DNNObject;
import org.firstinspires.ftc.teamcode.mentor.samples.ObjectDector.OPCVFFObjectDetector3;
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
import java.util.Random;

public class OpenCvShippingElementDetector extends OpenCvPipeline {



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
    Map<TSELocation, Integer> levelSamples = new HashMap<>();


    private int width; // width of the image
    private int height = 224;
    private double inScaleFactor = 0.007843;
    private double thresholdDnn =  0.2;
    private double meanVal = 127.5;
    private Dnn cvDNN = null;
    private Net net = null;
    private Telemetry telemetry = null;
    private Mat imageRGB = new Mat();

    //used to store maximum confidence amoung samples taken.  Only overwrite the level if we get a higher confidence
    private float maxConfidence = 0.0f;
    private boolean absolutelySure = false;

    static final private float CONF_THRESHOLD = 0.75f;

    private final String[] classNames = {"background",
            "duckside_blue_level_3", "duckside_blue_level_2", "duckside_blue_level_1", "warehouseside_blue_level_1", "warehouseside_blue_level_2", "warehouseside_blue_level_3" };

    private List<TSELocation> locationSamples = new ArrayList<>();


    //private static List<Scalar> colors=new ArrayList<>();

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public OpenCvShippingElementDetector(int width, int height, Telemetry telemetry) {
        this.width = width;
        this.height = height;
        this.telemetry = telemetry;

        levels.put(TSELocation.NONE,0);

        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_1,1);
        levels.put(TSELocation.WAREHOUSESIDE_BLUE_LEVEL_1,1);
        levels.put(TSELocation.DUCKSIDE_RED_LEVEL_1,1);
        levels.put(TSELocation.WAREHOUSESIDE_RED_LEVEL_1,1);

        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_2,2);
        levels.put(TSELocation.WAREHOUSESIDE_BLUE_LEVEL_2,2);
        levels.put(TSELocation.DUCKSIDE_RED_LEVEL_2,2);
        levels.put(TSELocation.WAREHOUSESIDE_RED_LEVEL_2,2);

        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_3,3);
        levels.put(TSELocation.WAREHOUSESIDE_BLUE_LEVEL_3,3);
        levels.put(TSELocation.DUCKSIDE_RED_LEVEL_3,3);
        levels.put(TSELocation.WAREHOUSESIDE_RED_LEVEL_3,3);

        levelSamples.put(TSELocation.NONE,0);

        levelSamples.put(TSELocation.DUCKSIDE_BLUE_LEVEL_1,0);
        levelSamples.put(TSELocation.WAREHOUSESIDE_BLUE_LEVEL_1,0);
        levelSamples.put(TSELocation.DUCKSIDE_RED_LEVEL_1,0);
        levelSamples.put(TSELocation.WAREHOUSESIDE_RED_LEVEL_1,0);

        levelSamples.put(TSELocation.DUCKSIDE_BLUE_LEVEL_2,0);
        levelSamples.put(TSELocation.WAREHOUSESIDE_BLUE_LEVEL_2,0);
        levelSamples.put(TSELocation.DUCKSIDE_RED_LEVEL_2,0);
        levelSamples.put(TSELocation.WAREHOUSESIDE_RED_LEVEL_2,0);

        levelSamples.put(TSELocation.DUCKSIDE_BLUE_LEVEL_3,0);
        levelSamples.put(TSELocation.WAREHOUSESIDE_BLUE_LEVEL_3,0);
        levelSamples.put(TSELocation.DUCKSIDE_RED_LEVEL_3,0);
        levelSamples.put(TSELocation.WAREHOUSESIDE_RED_LEVEL_3,0);

        location = TSELocation.NONE;


        cvDNN = new Dnn();
        net = cvDNN.readNetFromTensorflow("/sdcard/FIRST/EasyOpenCV/models/freight_frenzy_barcodes_graph.pb");
        //for(int i=0; i<classNames.length; i++)
            //colors.add(randomColor());

    }


    @Override
    public Mat processFrame(Mat inputFrame) {

        //telemetry.addLine("Inside ProcessFrame");
        //telemetry.update();
        //Mat imageRGB = new Mat();
        Mat blob = null;
        Mat detections = null;

        Imgproc.cvtColor(inputFrame,imageRGB,Imgproc.COLOR_RGBA2RGB);

        blob = Dnn.blobFromImage(imageRGB, inScaleFactor,
                new Size(224, 224),
                new Scalar(meanVal, meanVal, meanVal),
                false, true);

        net.setInput(blob);
        detections = net.forward();


        //blob.release();


        for (int i = 0; i < detections.rows(); ++i) {
            Mat row = detections.row(i);
            //Mat scores = row.colRange(5, detections.cols());
            Core.MinMaxLocResult mm = Core.minMaxLoc(row);
            //Core.MinMaxLocResult mm = Core.minMaxLoc(scores);
            float confidence = (float) mm.maxVal;
            Point classIdPoint = mm.maxLoc;


            if (confidence >= CONF_THRESHOLD) {



                int centerX = (int) (row.get(0, 0)[0] * row.cols());
                int centerY = (int) (row.get(0, 1)[0] * row.rows());
                int width = (int) (row.get(0, 2)[0] * row.cols());
                int height = (int) (row.get(0, 3)[0] * row.rows());

                //row.release();

                int left = (int) (centerX - width * 0.5);
                int top = (int) (centerY - height * 0.5);
                int right = (int) (centerX + width * 0.5);
                int bottom = (int) (centerY + height * 0.5);

                Point left_top = new Point(left, top);
                Point right_bottom = new Point(right, bottom);
                Point label_left_top = new Point(left, top + 30);
                DecimalFormat df = new DecimalFormat("#.##");

                int class_id = (int) classIdPoint.x;
                String className = classNames[class_id].toString();
                String label =  className + ": " + df.format(confidence);
                //Scalar color = colors.get(class_id);

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

                    case "warehouseside_blue_level_1":
                        location = TSELocation.WAREHOUSESIDE_BLUE_LEVEL_1;
                        break;

                    case "warehouseside_blue_level_2":
                        location = TSELocation.WAREHOUSESIDE_BLUE_LEVEL_2;
                        break;

                    case "warehouseside_blue_level_3":
                        location = TSELocation.WAREHOUSESIDE_BLUE_LEVEL_3;
                        break;

                    case "duckside_red_level_1":
                        location = TSELocation.DUCKSIDE_RED_LEVEL_1;
                        break;

                    case "duckside_red_level_2":
                        location = TSELocation.DUCKSIDE_RED_LEVEL_2;
                        break;

                    case "duckside_red_level_3":
                        location = TSELocation.DUCKSIDE_RED_LEVEL_3;
                        break;

                    case "warehouseside_red_level_1":
                        location = TSELocation.WAREHOUSESIDE_RED_LEVEL_1;
                        break;

                    case "warehouseside_red_level_2":
                        location = TSELocation.WAREHOUSESIDE_RED_LEVEL_2;
                        break;

                    case "warehouseside_red_level_3":
                        location = TSELocation.WAREHOUSESIDE_RED_LEVEL_3;
                        break;

                    default:
                        location = TSELocation.NONE;
                }
                int newVal = levelSamples.get(location) + 1;

                levelSamples.replace(location,newVal);

                telemetry.addData("The location being set is", location + " " + newVal);
                telemetry.addData("With confidence", confidence);

                telemetry.addData("This is a real location", getLocation());
                telemetry.update();

                //Imgproc.rectangle(imageRGB, left_top, right_bottom, color, 3, 2);
                Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(0, 0, 0), 4);
                Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(255, 255, 255), 2);
            }

        }

        //detections.release();

        return imageRGB;
    }

    /*private Scalar randomColor() {
        Random random = new Random();
        int r = random.nextInt(255);
        int g = random.nextInt(255);
        int b = random.nextInt(255);
        return new Scalar(r,g,b);

    }*/
    public TSELocation getLocation() {
        final Integer[] value = new Integer[1];
        value[0] = 0;
        TSELocation[] mostSample = new TSELocation[1];
        levelSamples.forEach((k,v)->{
            if(v > value[0]){
                value[0] = v;
                mostSample[0] = k;

            }

        });
        return mostSample[0];
    }
    public int getTSELevel(){
        //telemetry.addData("getTSELevel", location);
        return levels.get(location);
    }
}