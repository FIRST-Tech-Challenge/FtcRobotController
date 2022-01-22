package org.firstinspires.ftc.teamcode.cv.sims;


import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.internal.android.dex.util.FileUtils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.FileNotFoundException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Scanner;


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


    //private String[] classNames;

    private final String modelTSE = "/Users/alex/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_barcodes/duckside_blue/converted_keras_duckside_blue_images_2/optimized/freight_frenzy_barcodes_duckside_blue_graph.pb";
    private final String modelPathDuckSideBlue = "/Users/alex/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_barcodes/duckside_blue/converted_keras_duckside_blue/optimized/freight_frenzy_barcodes_duckside_blue_graph.pb";
    private final String modelPathDuckSideRed = "/Users/alex/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_barcodes/duckside_red/converted_keras_duckside_red/optimized/freight_frenzy_barcodes_duckside_red_graph.pb";
    private final String modelPathWarehouseSideBlue = "C:\\development\\BC4HStem\\FtcRobotController\\TeamCode\\src\\main\\assets\\tf_models\\freight_frenzy_barcodes\\warehouseside_blue\\converted_keras_warehouseside_blue\\optimized\\freight_frenzy_barcodes_warehouseside_blue_graph.pb";
    private final String modelPathWarehouseSideRed = "C:\\development\\BC4HStem\\FtcRobotController\\TeamCode\\src\\main\\assets\\tf_models\\freight_frenzy_barcodes\\warehouseside_blue\\converted_keras_warehouseside_blue\\optimized\\freight_frenzy_barcodes_warehouseside_blue_graph.pb";

    private String modelPath = modelTSE;
    private String modelPathConfig = modelTSE + "txt";

    private String labelsPath = "";

    //JSONObject dnnConfig = new JSONObject();

    private Mat blob = null;
    private Mat detections = null;
    private Mat row = null;

    List<Mat> result = new ArrayList<>();
    List<String> classNames = new ArrayList<>();



    public OpenCvShippingElementDetectorSim(Telemetry telemetry) {

        this.telemetry = telemetry;

        levels.put(TSELocation.NONE,0);

        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_1,1);
        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_2,2);
        levels.put(TSELocation.DUCKSIDE_BLUE_LEVEL_3,3);

        location = TSELocation.DUCKSIDE_BLUE_LEVEL_3;


        modelPath = modelPathDuckSideRed;
        modelPathConfig = modelPath + "txt";
        labelsPath = modelPath.substring(0, nthLastIndexOf(2, "/", modelPath)+1) + "labels.txt";
        inScaleFactor = 0.006; //0.0278;
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

        try{
            Scanner s = new Scanner(new File(labelsPath));

            while (s.hasNextLine()){
                classNames.add(s.nextLine());
            }
            s.close();
        }
        catch (FileNotFoundException ex){

        }


        cvDNN = new Dnn();
        net = cvDNN.readNetFromTensorflow(modelPath,modelPathConfig);
        net.setPreferableBackend(Dnn.DNN_BACKEND_DEFAULT);
        net.setPreferableTarget(Dnn.DNN_TARGET_CPU);
    }


    @Override
    public Mat processFrame(Mat inputFrame) {



        telemetry.addData("labels",classNames);
        // Show the bounding area in which we will search -
        Rect boundingRect = new Rect(50, 60, 180, 80);
        Imgproc.rectangle(inputFrame, boundingRect, new Scalar(0, 0, 255), 2); // BLUE

        Mat croppedMat = new Mat(inputFrame,boundingRect);
        Imgproc.resize(croppedMat,imageRGB,new Size(224,224),0,0);
        Imgproc.cvtColor(croppedMat,imageRGB,Imgproc.COLOR_BGR2RGB);

        telemetry.addData("imageRGB size",imageRGB.size());
        blob = Dnn.blobFromImage(imageRGB, inScaleFactor,
                new Size(224, 224),new Scalar(redMeanVal,greenMeanVal,blueMeanVal),false,cropDNN);

        //Mat blobMat = blob.reshape(blob.get(0,2) * blob.get(0,1), blob.shape[3], 1)
        //telemetry.addData("blob shape", blob.);
        net.setInput(blob);
        detections = net.forward();


        //detections = detections.reshape(1, (int) detections.total() / 7);
        //telemetry.addData("With ext con", detections.get(0,0));

        int cols = detections.cols();
        int rows = detections.rows();

        float[] result = new float[detections.size(0)*detections.size(1)];


        for (int i = 0; i < detections.rows(); ++i) {

            row = detections.row(i);

            Core.MinMaxLocResult mm = Core.minMaxLoc(row);
            int classId = (int) mm.maxLoc.x;

            float confidence = (float) mm.maxVal;


            telemetry.addData("mm max", mm.maxLoc);
            telemetry.addData("mm min", mm.minLoc);
            telemetry.addData("row total", detections.total());

            for(int j=0; j < row.cols(); ++j){

                double[] newRow = row.get(i, j);

                telemetry.addData("With col val", row.get(i, j)[0]);
            }
            if (confidence < CONF_THRESHOLD)
                continue;

            //int classId = (int)detections.get(i, 1)[0];
            //int classId = (int) row.get(i, 2)[0];

            //calculate position
            int left = 5;
            int top = 40;
            //int right = (int) (row.get(i, 2)[0] * (inputFrame.cols()));
            //int bottom = (int) (row.get(i, 3)[0] * (inputFrame.rows()));

            //Imgproc.circle(imageRGB,new Point(0,59),2,new Scalar(255,0,0),3);



            telemetry.addData("left", (int) left);
            telemetry.addData("top", (int) top);
            //telemetry.addData("right", (int) right);
            //telemetry.addData("bottom", (int) bottom);



            Point label_left_top = new Point(left, top - 5);
            DecimalFormat df = new DecimalFormat("#.##");

            //int class_id = (int) classIdPoint.x;
            String className = classNames.get(classId);
            String label =  className + ": " + df.format(confidence);

            //Imgproc.rectangle(imageRGB, left_top, right_bottom, new Scalar(0,0,255), 3, 2);
            Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(0, 0, 0), 4);
            Imgproc.putText(imageRGB, label, label_left_top, Imgproc.FONT_HERSHEY_SIMPLEX, .5, new Scalar(255, 255, 255), 2);


            telemetry.addData("With confidence", confidence);
            telemetry.addData("With Class", classId);
            //telemetry.addData("minLoc.x", mm.minLoc.x);




        }


        telemetry.update();
        return imageRGB;
    }




    public TSELocation getLocation() {
        return location;
    }
    public int getTSELevel(){
        //telemetry.addData("getTSELevel", location);
        return levels.get(location);
    }

    static int nthLastIndexOf(int nth, String ch, String string) {
        if (nth <= 0) return string.length();
        return nthLastIndexOf(--nth, ch, string.substring(0, string.lastIndexOf(ch)));
    }
}