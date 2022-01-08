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
    Map<TSELocation, Integer> levelSamples = new HashMap<>();


    public int width = 224; // width of the image
    public int height = 240;

    public double inScaleFactor = 0.0278;
    public double thresholdDnn =  0.6;
    public double redMeanVal = 5;
    public double greenMeanVal = 113;
    public double blueMeanVal = 119;

    public Point left_top = new Point(9,0);
    public Point right_bottom = new Point(40,40);

    private Dnn cvDNN = null;
    private Net net = null;
    private Telemetry telemetry = null;
    private Mat imageRGB = new Mat();

    private Scalar GREEN = new Scalar(0, 0, 255);

    //used to store maximum confidence amoung samples taken.  Only overwrite the level if we get a higher confidence
    private float maxConfidence = 0.0f;
    private boolean absolutelySure = false;

    static final private float CONF_THRESHOLD = 0.75f;



    private final String[] classNamesDuckSideBlue = {"background",
            "duckside_blue_level_3", "duckside_blue_level_2", "duckside_blue_level_1" };

    private final String[] classNamesDuckSideRed = {"background",
            "duckside_red_level_3", "duckside_red_level_2", "duckside_red_level_1" };

    private final String[] classNamesWarehouseSideBlue = {"background",
            "warehouseside_blue_level_3", "warehouseside_blue_level_2", "warehouseside_blue_level_1" };

    private final String[] classNamesWarehouseSideRed = {"background",
            "warehouseside_red_level_3", "warehouseside_red_level_2", "warehouseside_red_level_1" };

    private String[] classNames = classNamesDuckSideBlue;

    private final String modelPathDuckSideBlue = "/Users/alex/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_barcodes/duckside_blue/converted_keras_duckside_blue/optimized/freight_frenzy_barcodes_duckside_blue_graph.pb";
    private final String modelPathDuckSideRed = "/sdcard/FIRST/EasyOpenCV/models/freight_frenzy_barcodes_duckside_red_graph.pb";
    private final String modelPathWarehouseSideBlue = "/sdcard/FIRST/EasyOpenCV/models/freight_frenzy_barcodes_warehouseside_blue_graph.pb";
    private final String modelPathWarehouseSideRed = "/sdcard/FIRST/EasyOpenCV/models/freight_frenzy_barcodes_warehouseside_red_graph.pb";

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

        classNames = classNamesDuckSideBlue;
        modelPath = modelPathDuckSideBlue;

        /*if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE &&
                Side.getInstance().getPositionSide() == Side.PositionSide.DUCKSIDE){
            classNames = classNamesDuckSideBlue;
            modelPath = modelPathDuckSideBlue;
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE &&
                Side.getInstance().getPositionSide() == Side.PositionSide.WAREHOUSESIDE)
        {
            classNames = classNamesWarehouseSideBlue;
            modelPath = modelPathWarehouseSideBlue;
        }

        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED &&
                Side.getInstance().getPositionSide() == Side.PositionSide.DUCKSIDE)
        {
            classNames = classNamesDuckSideRed;
            modelPath = modelPathDuckSideRed;
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED &&
                Side.getInstance().getPositionSide() == Side.PositionSide.WAREHOUSESIDE)
        {
            classNames = classNamesWarehouseSideRed;
            modelPath = modelPathWarehouseSideRed;
        }*/

        cvDNN = new Dnn();
        net = cvDNN.readNetFromTensorflow(modelPath);


    }
    public OpenCvShippingElementDetectorSim(int width, int height, Telemetry telemetry) {
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

        classNames = classNamesDuckSideBlue;
        modelPath = modelPathDuckSideRed;

        /*if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE &&
                Side.getInstance().getPositionSide() == Side.PositionSide.DUCKSIDE){
            classNames = classNamesDuckSideBlue;
            modelPath = modelPathDuckSideBlue;
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE &&
                Side.getInstance().getPositionSide() == Side.PositionSide.WAREHOUSESIDE)
        {
            classNames = classNamesWarehouseSideBlue;
            modelPath = modelPathWarehouseSideBlue;
        }

        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED &&
                Side.getInstance().getPositionSide() == Side.PositionSide.DUCKSIDE)
        {
            classNames = classNamesDuckSideRed;
            modelPath = modelPathDuckSideRed;
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED &&
                Side.getInstance().getPositionSide() == Side.PositionSide.WAREHOUSESIDE)
        {
            classNames = classNamesWarehouseSideRed;
            modelPath = modelPathWarehouseSideRed;
        }*/

        cvDNN = new Dnn();
        net = cvDNN.readNetFromTensorflow(modelPath);


    }


    @Override
    public Mat processFrame(Mat inputFrame) {



        Imgproc.cvtColor(inputFrame,imageRGB,Imgproc.COLOR_RGBA2RGB);

        blob = Dnn.blobFromImage(imageRGB, inScaleFactor,
                new Size(width, height),
                new Scalar(redMeanVal, greenMeanVal, blueMeanVal),
                true, true);

        net.setInput(blob);
        detections = net.forward();


        //blob.release();


        for (int i = 0; i < detections.rows(); ++i) {
            row = detections.row(i);

            Core.MinMaxLocResult mm = Core.minMaxLoc(row);

            float confidence = (float) mm.maxVal;
            Point classIdPoint = mm.maxLoc;


            if (confidence >= CONF_THRESHOLD) {



                int centerX = (int) (row.get(0, 0)[0] * row.cols());
                int centerY = (int) (row.get(0, 1)[0] * row.rows());
                int width = (int) (row.get(0, 2)[0] * row.cols());
                int height = (int) (row.get(0, 3)[0] * row.rows());

                telemetry.addLine(String.format("%d %d %d %d,",centerX,centerY,width,height));

                int left = (int) (centerX - width * 0.5);
                int top = (int) (centerY - height * 0.5);
                int right = (int) (centerX + width * 0.5);
                int bottom = (int) (centerY + height * 0.5);

                telemetry.addLine(String.format("%d %d %d %d,",left,top,right,bottom));

                left_top = new Point(left, top + 30);
                right_bottom = new Point(right, bottom);
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

                Imgproc.rectangle(imageRGB, left_top, right_bottom, GREEN, 3, 2);
                Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(0, 0, 0), 4);
                Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(255, 255, 255), 2);
            }

        }

        //detections.release();

        return imageRGB;
    }


    public TSELocation getLocation() {final Integer[] value = new Integer[1];
        /*value[0] = 0;
        TSELocation[] mostSample = new TSELocation[1];
        levelSamples.forEach((k,v)->{
            if(v > value[0]){
                value[0] = v;
                mostSample[0] = k;

            }

        });
        return mostSample[0];*/
        return location;
    }
    public int getTSELevel(){
        //telemetry.addData("getTSELevel", location);
        return levels.get(location);
    }
}