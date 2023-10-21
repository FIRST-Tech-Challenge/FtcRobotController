package org.firstinspires.ftc.teamcode.Components.CV.Pipelines;

import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p1x;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p1y;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p21x;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p21y;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p22x;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p22y;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p2x;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p2y;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p31x;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p31y;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p32x;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.p32y;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

/**
 * Warren
 * This is the pipeline that is activated when the robot is initializing, it will determine which position the team prop is in
 */
@Config
public class BlueSpikeObserverPipeline extends OpenCvPipeline {
    ArrayList<double[]> frameList;



    /**
     * This will construct the pipeline
     */
    public BlueSpikeObserverPipeline() {
        frameList=new ArrayList<>();
    }

    /**
     * This will process the frame
     * will NOT log, all this done asynchronously
     * @param input inputted fram from camera
     * @return outputted frame from this function
     */
    @Override
    public Mat processFrame(Mat input) {
        Rect ROI1 = new Rect( //130 x 210, 60 x 120
                new Point(p1x,p1y),
                new Point(p2x,p2y));
        Rect ROI2 = new Rect( //130 x 210, 60 x 120
                new Point(p21x,p21y),
                new Point(p22x,p22y));
        Rect ROI3 = new Rect( //130 x 210, 60 x 120
                new Point(p31x,p31y),
                new Point(p32x,p32y));


        Mat cone = input.submat(ROI1);
        //these are blue
        double redValue = Core.sumElems(cone).val[2]/ROI1.area()/255;
        cone = input.submat(ROI2);
        double redValue2 = Core.sumElems(cone).val[2]/ROI2.area()/255;
        cone = input.submat(ROI3);
        double redValue3 = Core.sumElems(cone).val[2]/ROI3.area()/255;
        frameList.add(new double[]{redValue, redValue2, redValue3});
        if(frameList.size()>5) {
            frameList.remove(0);
        }
        cone.release();




        //release all the data
//        input.release();
//        mat.release();
        Scalar color = new Scalar(255,0,0);
        Imgproc.rectangle(input, ROI1, color, 5);
        Imgproc.rectangle(input, ROI2, color, 5);
        Imgproc.rectangle(input, ROI3, color, 5);
        return input;
    }

    /**
     * This will get the spike location
     * Logs spike location to general medium verbosity
     * @return spike's location
     */
    public int getPosition(){
        double[] sums = {0,0,0};
        for(int i=0;i<frameList.size()-1;i++){
            sums[0]+=frameList.get(i)[0];
            sums[1]+=frameList.get(i)[1];
            sums[2]+=frameList.get(i)[2];
        }
        if(sums[0]>sums[1]&&sums[0]>sums[2]){
            return 1;
        }else if(sums[1]>sums[2]){
            return 2;
        }else{
            return 3;
        }
    }
}