package org.firstinspires.ftc.teamcode.vision;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class JoshOpenCV extends OpenCvPipeline
{
   Telemetry telemetry;




   public Scalar lower = new Scalar(0, 0, 0);
   public Scalar upper = new Scalar(255, 255, 255);
   public int blur = 0;


   //private Mat ycrcbMat       = new Mat();
   //private Mat binaryMat      = new Mat();
   Mat gray = new Mat();
   @Override
   public void init(Mat firstFrame)
   {

   }

   public JoshOpenCV(Telemetry telemetry) {
      this.telemetry = telemetry;
   }
   @Override
   public Mat processFrame(Mat input)
   {
      /*
       * Converts our input mat from RGB to YCrCb.
       * EOCV ALWAYS returns RGB mats, so you'd
       * always convert from RGB to the color
       * space you want to use.
       *
       * Takes our "input" mat as an input, and outputs
       * to a separate Mat buffer "ycrcbMat"
       */
      // https://photo.stackexchange.com/a/133360
      double diagonal_fov = 55;
      double x_resolution = 320;
      double y_resolution = 240;
      double diagonal_resolution = Math.sqrt(Math.pow(x_resolution, 2) + Math.pow(y_resolution, 2));
      // number of degrees per pixel
      double pixel_angle = diagonal_fov/diagonal_resolution;
      //original size of object
      double original_size = 3.5;
      double x_center = x_resolution/2;

      telemetry.addData("pixel angle", pixel_angle);
      Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
      Imgproc.medianBlur(gray, gray, 5);
      Mat circles = new Mat();
      Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
              (double)gray.rows()/16, // change this value to detect circles with different distances to each other
              100.0, 30.0, 1, 1000);
      for (int x = 0; x < circles.cols(); x++) {
         telemetry.addData("=============================================", "SPACER");

         double[] c = circles.get(0, x);
         Point center = new Point(Math.round(c[0]), Math.round(c[1]));
         // circle center
         Imgproc.circle(gray, center, 1, new Scalar(0,100,100), 3, 8, 0 );
         // circle outline
         int radius = (int) Math.round(c[2]);
         Imgproc.circle(gray, center, radius, new Scalar(255,0,255), (x+1), 8, 0 );
         // find the y_distance to ball (in inches)
         double angle_of_view = radius * 2 * pixel_angle;
         double y_distance = original_size/(2*Math.tan(Math.toRadians(angle_of_view/2)));
         telemetry.addData("angle of view", angle_of_view);
         telemetry.addData("Index", x);
         telemetry.addData("Point", center);
         telemetry.addData("Radius", radius);
         telemetry.addData("Y Distance", y_distance);

         // find the x_distance to ball (in inches)
         double rel_dist_from_center = center.x - x_center;
         double theta = rel_dist_from_center * pixel_angle;
         double x_distance = y_distance * Math.tan(Math.toRadians(theta));
         telemetry.addData("X Distance", x_distance);


      }
      telemetry.update();
      Imgproc.rectangle(gray, new Point(x_center, 0), new Point(x_center, y_resolution), new Scalar(255, 255, 255));



      /*
       * This is where our thresholding actually happens.
       * Takes our "ycrcbMat" as input and outputs a "binary"
       * Mat to "binaryMat" of the same size as our input.
       * "Discards" all the pixels outside the bounds specified
       * by the scalars above (and modifiable with EOCV-Sim's
       * live variable tuner.)
       *
       * Binary meaning that we have either a 0 or 255 value
       * for every pixel.
       *
       * 0 represents our pixels that were outside the bounds
       * 255 represents our pixels that are inside the bounds
       */
      //Core.inRange(ycrcbMat, lower, upper, binaryMat);




      return gray;
   }


}
