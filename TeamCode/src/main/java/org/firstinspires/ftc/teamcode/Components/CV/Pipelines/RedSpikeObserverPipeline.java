package org.firstinspires.ftc.teamcode.Components.CV.Pipelines;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Double.max;
import static java.lang.Double.min;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class RedSpikeObserverPipeline extends OpenCvPipeline {
  ArrayList<double[]> frameList;
  ArrayList<Integer> pos;
  public static double p1x = 0,
      p1y = 240,
      p2x = 80,
      p2y = 390,
      p21x = 90,
      p21y = 400,
      p22x = 150,
      p22y = 460,
      p31x = 510,
      p31y = 370,
      p32x = 600,
      p32y = 440,
      threshhold = 0.3,

      // h3u and s3u: 71 and 90
      colour = 1,
      h1 = 0,
      h1H = 30,
      h2 = 160,
      h2H = 180;

  /** This will construct the pipeline */
  public RedSpikeObserverPipeline() {
    frameList = new ArrayList<>();
    pos = new ArrayList<>();
  }

  /**
   * This will process the frame will NOT log, all this done asynchronously
   *
   * @param input inputted fram from camera
   * @return outputted frame from this function
   */
  @Override
  public Mat processFrame(Mat input) {

    Rect ROI1 =
        new Rect( // 130 x 210, 60 x 120
            new Point(p1x, p1y), new Point(p2x, p2y));
    Rect ROI2 =
        new Rect( // 130 x 210, 60 x 120
            new Point(p21x, p21y), new Point(p22x, p22y));
    Rect ROI3 = new Rect(new Point(p31x, p31y), new Point(p32x, p32y));
    Mat cone = input.submat(ROI1);
    double[] sums = Core.sumElems(cone).val;
    double redValue = (sums[0]+sums[1]+sums[2]) / ROI1.area() / 255;
    cone = input.submat(ROI2);
    sums = Core.sumElems(cone).val;
    double redValue2 = (sums[0]+sums[1]+sums[2]) / ROI2.area() / 255;
    cone = input.submat(ROI3);
    sums = Core.sumElems(cone).val;
    double redValue3 = (sums[0]+sums[1]+sums[2]) / ROI3.area() / 255;
    frameList.add(new double[] {redValue, redValue2, redValue3});
    if (frameList.size() > 5) {
      frameList.remove(0);
    }
    cone.release();
    Scalar color = new Scalar(255, 0, 0);
    Imgproc.rectangle(input, ROI1, color, 5);
    Imgproc.rectangle(input, ROI2, color, 5);
    Imgproc.rectangle(input, ROI3, color, 5);
    return input;
  }

  /**
   * This will get the spike location Logs spike location to general medium verbosity
   *
   * @return spike's location
   */
  public int getPosition() {
      double[] sums = {0, 0, 0};
        for (int i = 0; i < min(frameList.size() - 1,1); i++) {
      sums[0] += frameList.get(0)[0];
      sums[1] += frameList.get(0)[1];
      sums[2] += frameList.get(0)[2];
        }
      double diffRatio  = (sums[2] - sums[1])/ Double.min(sums[2],sums[1]);
      packet.put("frameListSize", frameList.size());
      packet.put("cvThresh0", sums[0]);
      packet.put("cvThresh1", sums[1]);
      packet.put("cvThresh2", sums[2]);
      if(diffRatio>threshhold){
        pos.add(0);
      }
      else if(diffRatio<-threshhold){
        pos.add(1);
      }
      else{
        pos.add(2);
      }
      if(pos.size()>5){
        pos.remove(0);
      }
      double[] counters = new double[3];
      for(int i=0;i<pos.size();i++){
        if(pos.get(i)==0){
          counters[0]+=.5;
        }
        if(pos.get(i)==1){
          counters[1]+=1;
        }
        if(pos.get(i)==2){
          counters[2]+=2;
        }
      }

      if(counters[0]>max(counters[1],counters[2])){
        return 0;
      }
      else if(counters[1]>max(counters[2],counters[0])){
        return 1;
      }
      else{
        return 2;
      }
    }
}
