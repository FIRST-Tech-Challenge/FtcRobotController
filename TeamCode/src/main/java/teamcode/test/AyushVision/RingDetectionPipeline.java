package teamcode.test.AyushVision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import teamcode.League1.LeagueOnePowerShotAuto;
import teamcode.common.Debug;

public class RingDetectionPipeline extends OpenCvPipeline {
    Mat workingMat;


    //need to calibrate all these values
    //min sum values of the mats in each of the 3 cases
    private final double RINGS_ZERO_SUM = 0;
    private final double RINGS_ONE_SUM = 1;
    private final double RINGS_FOUR_SUM = 4;
    private NumRings position;


    @Override
    public Mat processFrame(Mat input) {
        workingMat = new Mat();
        input.copyTo(workingMat);
        if(workingMat.empty()){
            return input;
        }
        Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2YCrCb); //grayscale the image



        Mat rings = workingMat.submat(120, 150, 10, 50); //cuts a submat from the whole image of what we want

        Imgproc.rectangle(workingMat, new Rect(10, 120, 40, 30), new Scalar(0, 255, 0));

        //Imgproc.rectangle(workingMat, new Rect(10, 120, 40, 30), new Scalar(0, 255, 0)); //draws a rectangle for debugging
        // so we can see what the robot is looking at (the Submat)

        double ringsMatSum = Core.sumElems(rings).val[2]; //sums the mats contents and from that we can deduce based on the sum how much orange
        //is in the frame, using the conditional logic below we can deduce the stack size and therefore auto zone
        Debug.log(ringsMatSum);
        if(ringsMatSum >= RINGS_ZERO_SUM && ringsMatSum < RINGS_ONE_SUM){
            position = NumRings.ZERO;
        }else if(ringsMatSum >= RINGS_ONE_SUM && ringsMatSum < RINGS_FOUR_SUM){
            position = NumRings.ONE;
        }else if(ringsMatSum >= RINGS_FOUR_SUM){
            position = NumRings.FOUR;
        }


        return workingMat;
    }
}
