package org.firstinspires.ftc.teamcode.Components.CV.Pipelines;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Math.abs;
import static java.lang.Math.min;

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
 * Warren This is the pipeline that is activated when the robot is initializing, it will determine
 * which position the team prop is in
 */
@Config
public class RedSpikeObserverPipeline extends OpenCvPipeline {
  ArrayList<double[]> frameList;
  public static double p1x = 0,
      p1y = 390,
      p2x = 80,
      p2y = 540,
      p21x = 290,
      p21y = 390,
      p22x = 460,
      p22y = 540,
      p31x = 650,
      p31y = 390,
      p32x = 830,
      p32y = 540,
      threshhold = 1.0,

      // h3u and s3u: 71 and 90
      colour = 1,
      h1 = 0,
      h1H = 30,
      h2 = 160,
      h2H = 180;

  /** This will construct the pipeline */
  public RedSpikeObserverPipeline() {
    frameList = new ArrayList<>();
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
    Rect ROI3 = new Rect(new Point(p31x, p32y), new Point(p32x, p32y));
    Mat cone = input.submat(ROI1);
    double redValue = Core.sumElems(cone).val[0] / ROI1.area() / 255;
    cone = input.submat(ROI2);
    double redValue2 = Core.sumElems(cone).val[0] / ROI1.area() / 255;
    cone = input.submat(ROI3);
    double redValue3 = Core.sumElems(cone).val[0] / ROI1.area() / 255;
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
    for (int i = 0; i < frameList.size() - 1; i++) {
      sums[0] += frameList.get(i)[0];
      sums[1] += frameList.get(i)[1];
      sums[2] += frameList.get(i)[2];
    }
    double diffRatio = abs(sums[1] - sums[0]) / sums[0];
    packet.put("frameListSize", frameList.size());
    packet.put("cvThresh0", sums[0]);
    packet.put("cvThresh1", sums[1]);
    packet.put("diffRatio", diffRatio);
    if (sums[0] < sums[1] && sums[0] < sums[2]) {
      return 0;
    } else if (sums[1] < sums[2] && sums[1] < sums[0]) {
      return 1;
    } else {
      return 2;
    }
  }
}
