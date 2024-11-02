package org.firstinspires.ftc.teamcode.myUtil.computerVision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class halfCompVis extends OpenCvPipeline {
    Mat YCbCR = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    OpMode opMode;
    double leftavgfin;
    double rightavgfin;
    Mat output = new Mat();
    Scalar rectColor;
    public  String location;


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCbCR, Imgproc.COLOR_RGB2YCrCb);
//        opMode.telemetry.addLine("Pipeline Running");

//
        Rect leftRect = new Rect(1,1, 319,319);
        Rect rightRect = new Rect(320, 1, 319, 319);

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        leftCrop = YCbCR.submat(leftRect);
        rightCrop = YCbCR.submat(rightRect);

        Core.extractChannel(leftCrop,leftCrop,2);
        Core.extractChannel(rightCrop,rightCrop,2);

        Scalar leftAvg = Core.mean(rightCrop);
        Scalar rightAvg = Core.mean(leftCrop);

        leftavgfin = leftAvg.val[0];
        rightavgfin = rightAvg.val[0];
        if ((leftavgfin<132 && rightavgfin>132) || (leftavgfin>132 && rightavgfin<132)) {
            if (leftavgfin > rightavgfin) {
                opMode.telemetry.addLine("Left");
                location = "Left";
            } else {
                opMode.telemetry.addLine("Middle");
                location = "Middle";
            }
        }else{
            opMode.telemetry.addLine("Right");
            location = "Right";
        }
        opMode.telemetry.addData("Left Color", (int)leftavgfin);
        opMode.telemetry.addData("Right Color", (int)rightavgfin);

        opMode.telemetry.update();

        return output;

    }

    public halfCompVis(OpMode opMode, Scalar rectColor){
        super();
        this.opMode = opMode;
        this.rectColor = rectColor;
    }
}
