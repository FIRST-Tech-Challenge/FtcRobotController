package org.firstinspires.ftc.teamcode.cv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.Levels;
import org.firstinspires.ftc.teamcode.globals.Side;
import org.firstinspires.ftc.teamcode.mentor.samples.ObjectDector.DNNObject;
import org.firstinspires.ftc.teamcode.mentor.samples.ObjectDector.OPCVFFObjectDetector3;
import org.json.JSONException;
import org.json.JSONObject;
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
        DUCKSIDE_BLUE_LEVEL_3,
        DUCKSIDE_BLUE_LEVEL_2,
        DUCKSIDE_BLUE_LEVEL_1,
        NONE
    }

    //TSELocation location;
    //Map<Levels.TSELocation, Integer> levels = new HashMap<>();


    private int width = 224; // width of the image
    private int height = 240;
    private double inScaleFactor = 0.0278;

    public double redMeanVal = 5;
    public double greenMeanVal = 113;
    public double blueMeanVal = 119;
    private boolean cropDNN = true;
    private Dnn cvDNN = null;
    private Net net = null;
    private Telemetry telemetry = null;
    private Mat imageRGB = new Mat();

    //used to store maximum confidence amoung samples taken.  Only overwrite the level if we get a higher confidence
    private float maxConfidence = 0.0f;
    private boolean absolutelySure = false;

    static final private float CONF_THRESHOLD = 0.55f;



    private final String[] classNamesDuckSideBlue = {"background",
            "duckside_blue_level_3", "duckside_blue_level_2", "duckside_blue_level_1" };


    private String[] classNames;

    private final String modelPathDuckSideBlue = "/sdcard/FIRST/EasyOpenCV/models/freight_frenzy_barcodes_duckside_blue_graph.pb";
    private final String modelPathDuckSideRed = "/sdcard/FIRST/EasyOpenCV/models/freight_frenzy_barcodes_duckside_red_graph.pb";
    private final String modelPathWarehouseSideBlue = "/sdcard/FIRST/EasyOpenCV/models/freight_frenzy_barcodes_warehouseside_blue_graph.pb";
    private final String modelPathWarehouseSideRed = "/sdcard/FIRST/EasyOpenCV/models/freight_frenzy_barcodes_warehouseside_blue_graph.pb";

    private String modelPath = modelPathDuckSideBlue;
    private String modelPathConfig = modelPathDuckSideBlue + "txt";

    JSONObject dnnConfig = new JSONObject();

    private Mat blob = null;
    private Mat detections = null;
    private Mat row = null;


    /**
     *
     * @param width The width of the image (check your camera)
     */
    public OpenCvShippingElementDetector(int width, int height, Telemetry telemetry) {
        //this.width = width;
        //this.height = height;
        this.telemetry = telemetry;




        try {
            JSONObject dsb = new JSONObject();
            dsb.put("modelPath",modelPathDuckSideBlue);
            dsb.put("modelPathConfig",modelPathDuckSideBlue + "txt");
            dsb.put("inScaleFactor",0.0278);
            dsb.put("redMeanVal",5);
            dsb.put("greenMeanVal",191.3);
            dsb.put("blueMeanVal",226.7);
            dsb.put("crop",true);

            JSONObject dsr = new JSONObject();
            dsr.put("modelPath",modelPathDuckSideRed);
            dsr.put("modelPathConfig",modelPathDuckSideRed + "txt");
            dsr.put("inScaleFactor",0.02);
            dsr.put("redMeanVal",24.1);
            dsr.put("greenMeanVal",154.4);
            dsr.put("blueMeanVal",144.5);
            dsr.put("crop",true);

            JSONObject wsb = new JSONObject();
            wsb.put("modelPath",modelPathWarehouseSideBlue);
            wsb.put("modelPathConfig",modelPathWarehouseSideBlue + "txt");
            wsb.put("inScaleFactor",0.02);
            wsb.put("redMeanVal",218.2);
            wsb.put("greenMeanVal",154.4);
            wsb.put("blueMeanVal",144.5);
            wsb.put("crop",true);

            JSONObject wsr = new JSONObject();
            wsr.put("modelPath",modelPathWarehouseSideRed);
            wsr.put("modelPathConfig",modelPathWarehouseSideRed + "txt");
            wsr.put("inScaleFactor",0.019);
            wsr.put("redMeanVal",87.8);
            wsr.put("greenMeanVal",65.2);
            wsr.put("blueMeanVal",144.5);
            wsr.put("crop",true);

            dnnConfig.put("dsbConfig",dsb);
            dnnConfig.put("dsrConfig",dsr);
            dnnConfig.put("wsb",wsb);
            dnnConfig.put("wsr",wsr);
        }
        catch (JSONException ex){

        }


        classNames = classNamesDuckSideBlue;
        JSONObject config = new JSONObject();
        try{
            config = dnnConfig.getJSONObject("dsb");
        }
        catch (JSONException ex){}


        if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE &&
                Side.getInstance().getPositionSide() == Side.PositionSide.DUCKSIDE){


            try{
                config = dnnConfig.getJSONObject("dsb");

            }
            catch (JSONException ex)
            {

            }

        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE &&
                Side.getInstance().getPositionSide() == Side.PositionSide.WAREHOUSESIDE)
        {
            try{
                config = dnnConfig.getJSONObject("wsb");

            }
            catch (JSONException ex)
            {

            }
        }

        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED &&
                Side.getInstance().getPositionSide() == Side.PositionSide.DUCKSIDE)
        {
            try{
                config = dnnConfig.getJSONObject("dsr");

            }
            catch (JSONException ex)
            {

            }
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED &&
                Side.getInstance().getPositionSide() == Side.PositionSide.WAREHOUSESIDE)
        {
            try{
                config = dnnConfig.getJSONObject("wsr");

            }
            catch (JSONException ex)
            {

            }
        }

        try{
            modelPath = config.getString("modelPath");
            modelPathConfig = config.getString("modelPathConfig");
            inScaleFactor = config.getDouble("inScaleFactor");
            redMeanVal = config.getDouble("redMeanVal");
            greenMeanVal = config.getDouble("greenMeanVal");
            blueMeanVal = config.getDouble("blueMeanVal");
            cropDNN = config.getBoolean("crop");
        }
        catch (JSONException ex){

        }


        net = Dnn.readNetFromTensorflow(modelPath);


    }


    @Override
    public Mat processFrame(Mat inputFrame) {



        Imgproc.cvtColor(inputFrame,imageRGB,Imgproc.COLOR_BGR2RGB);

        blob = Dnn.blobFromImage(imageRGB, inScaleFactor,
                new Size(width, height),
                new Scalar(redMeanVal, greenMeanVal, blueMeanVal),
                false, cropDNN);

        net.setInput(blob);
        detections = net.forward();


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

                //telemetry.addData("This is a real new", label);
                //telemetry.update();

                switch (className)
                {
                    case "duckside_blue_level_1":
                        Levels.getInstance().setTSELocation(Levels.TSELocation.LEVEL_1);
                        break;

                    case "duckside_blue_level_2":
                        Levels.getInstance().setTSELocation(Levels.TSELocation.LEVEL_2);
                        break;

                    case "duckside_blue_level_3":
                        Levels.getInstance().setTSELocation(Levels.TSELocation.LEVEL_3);
                        break;

                    default:
                        Levels.getInstance().setTSELocation(Levels.TSELocation.NONE);
                }

                telemetry.addData("The location being set is", Levels.getInstance().getTSELocation());
                telemetry.addData("With confidence", confidence);

                telemetry.addData("This is a real location", getLocation());
                telemetry.update();

                //Imgproc.rectangle(imageRGB, left_top, right_bottom, color, 3, 2);
                Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(0, 0, 0), 4);
                Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(255, 255, 255), 2);
            }

        }

        return imageRGB;
    }


    public Levels.TSELocation getLocation() {
        return Levels.getInstance().getTSELocation();
    }
    public int getTSELevel(){
        //telemetry.addData("getTSELevel", location);
        return Levels.getInstance().getTSELevel();
    }
}