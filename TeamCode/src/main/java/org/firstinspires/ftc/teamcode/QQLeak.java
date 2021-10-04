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
    Rect space1 = new Rect(8, 110, 50, 50);
    Mat slot1;
    Rect space2 = new Rect(126, 120, 50, 50);
    Mat slot2;
    Rect space3 = new Rect(230, 120, 50, 50);
    Mat slot3;
    int slotSelected = -1;

    public QQLeak(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public int getSlotSelected(){
        return slotSelected;
    }


    @Override
    public Mat processFrame(Mat inputMat) {
        Imgproc.cvtColor(inputMat, inputMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(inputMat, inputMat, new Size(3, 3), 0);
        Imgproc.rectangle(inputMat, space1, new Scalar(0, 0, 255), 3);
        Imgproc.rectangle(inputMat, space2, new Scalar(0, 0, 255), 3);
        Imgproc.rectangle(inputMat, space3, new Scalar(0, 0, 255), 3);
        slot1 = inputMat.submat(space1);
        slot2 = inputMat.submat(space2);
        slot3 = inputMat.submat(space3);
        double space1Color = Core.mean(slot1).val[1];
        telemetry.addData("Space 1", space1Color);
        double space2Color = Core.mean(slot2).val[1];
        telemetry.addData("Space 2", space2Color);
        double space3Color = Core.mean(slot3).val[1];

        if(space1Color >= space2Color && space1Color >= space3Color){
            slotSelected = 1;
            Imgproc.rectangle(inputMat, space1, new Scalar(0, 255, 0), 3);
        } else if (space2Color >= space3Color && space2Color >= space1Color ){
            slotSelected = 2;
            Imgproc.rectangle(inputMat, space2, new Scalar(0, 255, 0), 3);
        } else if (space3Color >= space2Color){
            slotSelected = 3;
            Imgproc.rectangle(inputMat, space3, new Scalar(0, 255, 0), 3);
        }

        telemetry.addData("Selected", slotSelected);

        telemetry.update();
        return inputMat;
    }

}
