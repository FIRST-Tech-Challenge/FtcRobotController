package org.firstinspires.ftc.teamcode.Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipelineBlue extends Pipeline{

    PipelineBlue (OpenCvCamera webcam, Telemetry telemetry){
        super(webcam, telemetry);
            this.webcam = webcam;
            this.telemetry = telemetry;
        }

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);//из RGB в YCrCb
            telemetry.addLine("Pipeline running");

            Rect leftRect = new Rect(0,280, 100, 100);
            Rect midleRect = new Rect(240,280, 300, 100);

            Core.extractChannel(YCbCr, YCbCr, 1);//оставляем только СИНИЙ цвет по политре YCbCr

            Imgproc.threshold(YCbCr, YCbCr, 120, 255, Imgproc.THRESH_BINARY_INV);

            leftCrop = YCbCr.submat(leftRect);
            midleCrop = YCbCr.submat(midleRect);


            double valueleft = Core.sumElems(leftCrop).val[0] / leftRect.area() / 255;
            double valuemiddle = Core.sumElems(midleCrop).val[0] / midleRect.area() / 255;

            leftCrop.release();
            midleCrop.release();


            //Процент нужного цвета в рамке
            telemetry.addData("Blue percentage in left", Math.round(valueleft * 100) + "%");
            telemetry.addData("Blue percentage in middle", Math.round(valuemiddle * 100) + "%");

            Imgproc.rectangle(YCbCr, leftRect, rectColor, 2);
            Imgproc.rectangle(YCbCr, midleRect, rectColor, 2);

            if(Math.round(valueleft * 100) > 10 && Math.round(valueleft * 100)> Math.round(valuemiddle * 100)){
                telemetry.addLine("Position: left");
            }else if(Math.round(valuemiddle * 100) > 10 && Math.round(valuemiddle * 100) > Math.round(valueleft * 100) ){
                telemetry.addLine("Position: middle");
            }else {
                telemetry.addLine("Position: right");
            }

            telemetry.update();

            return (YCbCr);
        }

}
