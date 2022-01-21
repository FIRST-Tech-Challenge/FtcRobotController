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
        DUCKSIDE_BLUE_LEVEL_3,
        DUCKSIDE_BLUE_LEVEL_2,
        DUCKSIDE_BLUE_LEVEL_1,
        NONE
    }

    TSELocation location;
    Map<TSELocation, Integer> levels = new HashMap<>();


    private int width = 224; // width of the image
    private int height = 224;
    public double inScaleFactor = 0; //0.0278;

    public double redMeanVal = 255; //5;
    public double greenMeanVal = 255; //113;
    public double blueMeanVal = 255; //119;
    public boolean cropDNN = true;
    private Dnn cvDNN = null;
    private Net net = null;
    private Telemetry telemetry = null;
    private Mat imageRGB = new Mat();

    //used to store maximum confidence amoung samples taken.  Only overwrite the level if we get a higher confidence
    private float maxConfidence = 0.0f;
    private boolean absolutelySure = false;

    public float CONF_THRESHOLD = 0.55f;



    private final String[] classNamesDuckSideBlue = {"background",
            "duckside_blue_level_3", "duckside_blue_level_2", "duckside_blue_level_1" };


    private String[] classNames;

    private final String modelTSE = "/Users/alex/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_barcodes/tse_giant_sensor_converted_keras/optimized/freight_frenzy_barcodes_tse_giants_sensor_graph.pb";
    private final String modelPathDuckSideBlue = "/Users/alex/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_barcodes/duckside_blue/converted_keras_duckside_blue/optimized/freight_frenzy_barcodes_duckside_blue_graph.pb";
    private final String modelPathDuckSideRed = "C:\\development\\BC4HStem\\FtcRobotController\\TeamCode\\src\\main\\assets\\tf_models\\freight_frenzy_barcodes\\duckside_red\\converted_keras_duckside_red\\optimized\\freight_frenzy_barcodes_duckside_red_graph.pb";
    private final String modelPathWarehouseSideBlue = "C:\\development\\BC4HStem\\FtcRobotController\\TeamCode\\src\\main\\assets\\tf_models\\freight_frenzy_barcodes\\warehouseside_blue\\converted_keras_warehouseside_blue\\optimized\\freight_frenzy_barcodes_warehouseside_blue_graph.pb";
    private final String modelPathWarehouseSideRed = "C:\\development\\BC4HStem\\FtcRobotController\\TeamCode\\src\\main\\assets\\tf_models\\freight_frenzy_barcodes\\warehouseside_blue\\converted_keras_warehouseside_blue\\optimized\\freight_frenzy_barcodes_warehouseside_blue_graph.pb";

    private String modelPath = modelTSE;
    private String modelPathConfig = modelTSE + "txt";

    //JSONObject dnnConfig = new JSONObject();

    private Mat blob = null;
    private Mat detections = null;
    private Mat row = null;

    List<Mat> result = new ArrayList<>();

    public OpenCvShippingElementDetectorSim(Telemetry telemetry) {

        this.telemetry = telemetry;

        levels.put(TSELocation.NONE,0);

        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_1,1);
        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_2,2);
        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_3,3);

        location = TSELocation.DUCKSIDE_BLUE_LEVEL_3;

        classNames = classNamesDuckSideBlue;
        modelPath = modelTSE;
        modelPathConfig = modelPath + "txt";
        inScaleFactor = 1; //0.0278;
        redMeanVal = 127.5; //5;
        greenMeanVal = 127.5; //113;
        blueMeanVal = 127.5; //119;
        cropDNN = true;

        /*modelPath = modelPathDuckSideRed;
        modelPathConfig = modelPathDuckSideRed + "txt";
        inScaleFactor = 0.02;
        redMeanVal = 24.1;
        greenMeanVal = 154.4;
        blueMeanVal = 144.5;
        cropDNN = true;*/

        /*modelPath = modelPathWarehouseSideBlue;
        modelPathConfig = modelPathWarehouseSideBlue + "txt";
        inScaleFactor = 0.02;
        redMeanVal = 218.2;
        greenMeanVal = 154.4;
        blueMeanVal = 144.5;
        cropDNN = true;*/

        /*modelPath = modelPathWarehouseSideRed;
        modelPathConfig = modelPathWarehouseSideRed + "txt";
        inScaleFactor = 0.019;
        redMeanVal = 87.8;
        greenMeanVal = 65.2;
        blueMeanVal = 144.5;
        cropDNN = true;*/

        /*try {
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
            wsr.put("modelPath",modelPathWarehouseSideBlue);
            wsr.put("modelPathConfig",modelPathWarehouseSideBlue + "txt");
            wsr.put("inScaleFactor",0.02);
            wsr.put("redMeanVal",158.7);
            wsr.put("greenMeanVal",79.3);
            wsr.put("blueMeanVal",97.8);
            wsr.put("crop",true);

            dnnConfig.put("dsbConfig",dsb);
            dnnConfig.put("dsrConfig",dsr);
            dnnConfig.put("wsb",wsb);
            dnnConfig.put("wsr",wsr);
        }
        catch (JSONException ex){

        }*/



        /*JSONObject config = new JSONObject();
        try{
            config = dnnConfig.getJSONObject("dsb");
        }
        catch (JSONException ex){}


        /*if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE &&
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
        }*/

        /*try{
            modelPath = config.getString("modelPath");
            modelPathConfig = config.getString("modelPathConfig");
            inScaleFactor = config.getDouble("inScaleFactor");
            redMeanVal = config.getDouble("redMeanVal");
            greenMeanVal = config.getDouble("greenMeanVal");
            blueMeanVal = config.getDouble("blueMeanVal");
            cropDNN = config.getBoolean("crop");
        }
        catch (JSONException ex){

        }*/

        cvDNN = new Dnn();
        net = cvDNN.readNetFromTensorflow(modelPath);



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

        int cols = imageRGB.cols();
        int rows = imageRGB.rows();

        //detections = detections.reshape(1, (int) detections.total() / 7);
        //telemetry.addData("With ext con", detections.get(0,0));

        double[] detect = detections.get(0,0);
        telemetry.addData("With ext con", (int) (detections.get(0, 0)[0] * inputFrame.rows()));

        for (int i = 0; i < detections.rows(); ++i) {

            row = detections.row(i);
            telemetry.addData("With ext con", (int) (row.get(0, 0)[0] * inputFrame.cols()));
            Core.MinMaxLocResult mm = Core.minMaxLoc(row);

            float confidence = (float) mm.maxVal;

            //double confidence = detections.get(i, 2)[0];
            //telemetry.addData("With det confidence", detections.get(i, 2)[0]);

            telemetry.addData("mm max", mm.maxLoc);
            telemetry.addData("mm min", mm.minLoc);
            telemetry.addData("row total", detections.total());
            //row.reshape(1, (int) row.total() / 7);
            for(int j=0; j < row.cols(); ++j){

                telemetry.addData("With col val", row.get(i, j)[0]);
            }
            if (confidence < CONF_THRESHOLD)
                continue;

            int classId = (int) detections.get(i, 1)[0];

            //calculate position
            int centerX = (int) (row.get(i, 0)[0] * imageRGB.cols());
            int centerY = (int) (row.get(i, 1)[0] * imageRGB.rows());
            int width = (int) (row.get(i, 2)[0] * imageRGB.cols());
            int height = (int) (row.get(i, 3)[0] * imageRGB.rows());

            telemetry.addData("centerX", (int) centerX);
            telemetry.addData("centerY", (int) centerY);
            telemetry.addData("width", (int) width);
            telemetry.addData("height", (int) height);

            int left = (int) (centerX - width * 0.5);
            int top = (int) (centerY - height * 0.5);
            int right = (int) (centerX + width * 0.5);
            int bottom = (int) (centerY + height * 0.5);

            Point left_top = new Point(left, top);
            Point right_bottom = new Point(right, bottom);
            Point label_left_top = new Point(left, top - 5);
            DecimalFormat df = new DecimalFormat("#.##");

            //int class_id = (int) classIdPoint.x;
            String className = classNames[classId].toString();
            String label =  className + ": " + df.format(confidence);

            Imgproc.rectangle(imageRGB, left_top, right_bottom, new Scalar(0,0,255), 3, 2);
            Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(0, 0, 0), 4);
            Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(255, 255, 255), 2);


            telemetry.addData("With confidence", confidence);
            telemetry.addData("With Class", classId);
            //telemetry.addData("minLoc.x", mm.minLoc.x);




        }

        /*for (int i = 0; i < result.size(); ++i) {
            // each row is a candidate detection, the 1st 4 numbers are
            // [center_x, center_y, width, height], followed by (N-4) class probabilities
            Mat level = result.get(i);
            for (int j = 0; j < level.rows(); ++j) {
                Mat row = level.row(j);
                Mat scores = row.colRange(4, level.cols());


                Core.MinMaxLocResult mm = Core.minMaxLoc(level);
                float confidence = (float) mm.maxVal;
                Point classIdPoint = mm.maxLoc;

                telemetry.addData("confidence", confidence);
                telemetry.addData("With Class", mm.maxLoc);
                telemetry.addData("minLoc.x", mm.minLoc.x);
                telemetry.update();

                if (confidence > CONF_THRESHOLD) {

                    int centerX = (int) (row.get(0, 0)[0] * imageRGB.cols());
                    int centerY = (int) (row.get(0, 1)[0] * imageRGB.rows());
                    int width = (int) (row.get(0, 2)[0] * imageRGB.cols());
                    int height = (int) (row.get(0, 3)[0] * imageRGB.rows());

                    int left = (int) (centerX - width * 0.5);
                    int top =(int)(centerY - height * 0.5);
                    int right =(int)(centerX + width * 0.5);
                    int bottom =(int)(centerY + height * 0.5);

                    Point left_top = new Point(left, top);
                    Point right_bottom=new Point(right, bottom);
                    Point label_left_top = new Point(left, top-5);
                    DecimalFormat df = new DecimalFormat("#.##");

                    int class_id = (int) classIdPoint.x;
                    //String label= classNames[class_id] + ": " + df.format(confidence);
                    //Scalar color= colors.get(class_id);

                    //Imgproc.rectangle(imageRGB, left_top,right_bottom , new Scalar(0,255,0), 3, 2);
                    //Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 0, 0), 4);
                    //Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255), 2);
                }
            }
        }*/
        telemetry.update();
        return imageRGB;
    }

        /*for (int i = 0; i < detections.rows(); ++i) {
            row = detections.row(i);

            Core.MinMaxLocResult mm = Core.minMaxLoc(row);

            float confidence = (float) mm.maxVal;
            Point classIdPoint = mm.maxLoc;

            telemetry.addData("With confidence", confidence);
            telemetry.addData("With Class", mm.maxLoc);
            telemetry.addData("minLoc.x", mm.minLoc.x);
            //telemetry.update();

            if (confidence >= CONF_THRESHOLD) {



                int centerX = (int) (row.get(0, 0)[0] * imageRGB.cols());
                int centerY = (int) (row.get(0, 1)[0] * imageRGB.rows());
                int width = (int) (row.get(0, 2)[0] * imageRGB.cols());
                int height = (int) (row.get(0, 3)[0] * imageRGB.rows());

                //row.release();

                int left = (int) (centerX - width * 0.5);
                int top = (int) (centerY - height * 0.5);
                int right = (int) (centerX + width * 0.5);
                int bottom = (int) (centerY + height * 0.5);

                Point left_top = new Point(left, top);
                Point right_bottom = new Point(right, bottom);
                Point label_left_top = new Point(left, top - 20);
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

                Imgproc.rectangle(imageRGB, left_top, right_bottom, new Scalar(0,0,255), 3, 2);
                Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(0, 0, 0), 4);
                Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(255, 255, 255), 2);
            }

        }

        return imageRGB;
    }*/


    public TSELocation getLocation() {
        return location;
    }
    public int getTSELevel(){
        //telemetry.addData("getTSELevel", location);
        return levels.get(location);
    }
}