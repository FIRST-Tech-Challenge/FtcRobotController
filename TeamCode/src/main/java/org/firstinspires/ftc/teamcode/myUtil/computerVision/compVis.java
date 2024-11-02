package org.firstinspires.ftc.teamcode.myUtil.computerVision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class compVis extends OpenCvPipeline {

    Mat YCbCR = new Mat();
    Mat leftCrop;
    Mat midCrop;
    Mat rightCrop;
    Scalar rectColor;// = new Scalar(255.0,0.0,0.0);
    Mat outPut = new Mat();
    double leftavfin;
    double midavfin;
    double rightavfin;
    Rect leftRect;
    Rect midRect;
    Rect rightRect;
    Scalar leftavg;
    Scalar midavg;
    Scalar rightavg;
    OpMode opMode;
    int frames = -1;
    String splash = "";
    Colors color;
    Hardware r;
    public enum Colors{
        RED,
        BLUE
    }
    String[] splashes = {
            "please check on your coder",
            "deleting the evidence",
            "c0rrup71ng da7a",
            "hacking opponents",
            "everything is going smoothly",
            "20325 Pride!",
            "if this is not on the 20325 robot, it is stolen",
            "I'll be back"
    };
    public  String loc = "";
    public compVis(OpMode opMode, Colors color, Hardware r){
        super();
        this.opMode = opMode;
        this.rectColor = new Scalar(255,0,0);
        this.r = r;
        this.color = color;

    }

    @Override
    public Mat processFrame(Mat input) {
        r.waiter(10);
        frames += 1;
        frames %= 1000;
        Imgproc.cvtColor(input,YCbCR, Imgproc.COLOR_RGB2YCrCb);

            leftRect = new Rect(5, 0, 111, 359);
            midRect = new Rect(191, 0, 270, 359);
            rightRect = new Rect(550, 0, 78, 359);


            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, midRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);


            leftCrop = YCbCR.submat(leftRect);
            midCrop = YCbCR.submat(midRect);
            rightCrop = YCbCR.submat(rightRect);


            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(midCrop, midCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);


            leftavg = Core.mean(leftCrop);
            midavg = Core.mean(midCrop);
            rightavg = Core.mean(rightCrop);

            leftavfin = leftavg.val[0];
            midavfin = midavg.val[0];
            rightavfin = rightavg.val[0];
            if (color == Colors.RED) {
                leftavfin = 200 - leftavfin;
                midavfin = 200 - midavfin;
                rightavfin = 200 - rightavfin;
            }
//
//
//
        opMode.telemetry.addData("Left", (int)leftavfin);
        opMode.telemetry.addData("Mid", (int)midavfin);
        opMode.telemetry.addData("Right", (int)rightavfin);

        if ((int)leftavfin > (int)rightavfin){
            if ((int)leftavfin > (int)midavfin){
                loc = "Left";
            }else{
                loc = "Mid";
            }
        }else{
            if ((int)midavfin < (int)rightavfin){
                loc = "Right";
            }else{
                loc = "Mid";
            }
        }
        if (frames == 0){
            splash =splashes[(int)(Math.random()* (splashes.length))];
        }
        opMode.telemetry.addLine("Frame is being processed, " + splash);
        opMode.telemetry.addData("Location", loc);
        opMode.telemetry.update();

            return outPut;



//        return null;
    }
}
