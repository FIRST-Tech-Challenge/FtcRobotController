package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Size;
import org.opencv.core.Rect;

public class QQLeak extends OpenCvPipeline {
    //they very kindly didn't insert comments...-_-
    Telemetry telemetry;
    Mat hsv = new Mat();
    Mat output = new Mat();
    Rect space1 = new Rect(0, 140, 200, 200);//barcode position 1 i think
    Mat slot1;
    Rect space2 = new Rect(220, 140, 200, 200);//so this is position 2 no matter what
    Mat slot2;
    Rect space3 = new Rect(440, 140, 200, 200);//position 3 maybe?
    Mat slot3;
    int slotSelected = -1;//for storing which location the duck/thing is in

    public QQLeak(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public int getSlotSelected(){
        return slotSelected;
    }


    @Override
    public Mat processFrame(Mat inputMat) {
        inputMat.copyTo(output);
        Imgproc.cvtColor(output, hsv, Imgproc.COLOR_BGR2HSV_FULL);//change color to hsv
        Imgproc.GaussianBlur(hsv, hsv, new Size(5, 5), 0);//blur image
        Imgproc.rectangle(output, space1, new Scalar(0, 0, 255), 3);//draw a rectangle around each area???
        Imgproc.rectangle(output, space2, new Scalar(0, 0, 255), 3);
        Imgproc.rectangle(output, space3, new Scalar(0, 0, 255), 3);
        slot1 = hsv.submat(space1);//make some submats
        slot2 = hsv.submat(space2);
        slot3 = hsv.submat(space3);
        double space1Color = Core.mean(slot1).val[1];//find some color thing??????
        telemetry.addData("Space 1", space1Color);
        double space2Color = Core.mean(slot2).val[1];
        telemetry.addData("Space 2", space2Color);
        double space3Color = Core.mean(slot3).val[1];
        telemetry.addData("Space 3", space3Color);

        /*if(space1Color >= space2Color && space1Color >= space3Color){
            slotSelected = 1;
            Imgproc.rectangle(inputMat, space1, new Scalar(0, 255, 0), 3);
        } else if (space2Color >= space3Color && space2Color >= space1Color ){
            slotSelected = 2;
            Imgproc.rectangle(inputMat, space2, new Scalar(0, 255, 0), 3);
        } else if (space3Color >= space2Color){
            slotSelected = 3;
            Imgproc.rectangle(inputMat, space3, new Scalar(0, 255, 0), 3);
        }*/

        double diff1 = Math.abs(space1Color-40);
        double diff2 = Math.abs(space2Color-40);
        double diff3 = Math.abs(space3Color-40);

        if (diff1 <= diff2 && diff1 <= diff3) {
            slotSelected = 1;
            Imgproc.rectangle(output, space1, new Scalar(0, 255, 0), 3);
        } else if (diff2 <= diff1 && diff2 <= diff3) {
            slotSelected = 2;
            Imgproc.rectangle(output, space2, new Scalar(0, 255, 0), 3);
        } else if (diff3 <= diff1 && diff3 <= diff2) {
            slotSelected = 3;
            Imgproc.rectangle(output, space3, new Scalar(0, 255, 0), 3);
        }

        telemetry.addData("Selected", slotSelected);

        telemetry.update();
        return output;
    }

}
