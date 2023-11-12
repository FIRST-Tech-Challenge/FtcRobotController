package org.firstinspires.ftc.teamcode.Components.CV.Pipelines;

import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.h1;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.h1H;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.h2H;
import static org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RedSpikeObserverPipeline.threshhold;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

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
    public static double p1x = 500, p1y =230, p2x = 640, p2y =380, p21x = 150, p21y =200, p22x = 350, p22y =370;



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
        Mat cone = input.submat(ROI1);
        Mat newCone = new Mat();
        Imgproc.cvtColor(cone, newCone, Imgproc.COLOR_RGB2HSV);
        Scalar thresh1 = new Scalar(h1,59,0), hthresh1 = new Scalar(h1H,255,255);
        Scalar thresh2 = new Scalar(RedSpikeObserverPipeline.h2,59,0), hthresh2 = new Scalar(h2H,255,255);
        Mat filtered = new Mat(), filtered1 = new Mat();
        Core.inRange(newCone, thresh1, hthresh1, filtered);
        Core.inRange(newCone, thresh2, hthresh2, filtered1);
        Mat thresh = new Mat();
        Core.add(filtered1, filtered, thresh);
        double redValue = Core.sumElems(thresh).val[0]/ROI1.area()/255;
        cone = input.submat(ROI2);
        newCone = new Mat();
        Imgproc.cvtColor(cone, newCone, Imgproc.COLOR_RGB2HSV);
        filtered = new Mat();
        filtered1 = new Mat();
        Core.inRange(newCone, thresh1, hthresh1, filtered);
        Core.inRange(newCone, thresh2, hthresh2, filtered1);
        thresh = new Mat();
        Core.add(filtered1, filtered, thresh);
        double redValue2 = Core.sumElems(thresh).val[0]/ROI2.area()/255;
        frameList.add(new double[]{redValue, redValue2});
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
        }
        packet.put("frameListSize", frameList.size());
        packet.put("cvThresh0", sums[0]);
        packet.put("cvThresh1", sums[1]);

        if(sums[0]> threshhold&&sums[1]>threshhold){
            if(sums[0]>sums[1]){
                return 3;
            }
            else{
                return 2;
            }
        }else if(sums[1]>threshhold){
            return 2;
        } else if(sums[0]>threshhold){
            return 3;
        }else{
            return 1;
        }
    }
}