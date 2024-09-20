package org.firstinspires.ftc.teamcode.vision;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class opencvtest2 extends OpenCvPipeline
{
   //Telemetry telemetry;


   public enum SkystonePosition
   {
      LEFT,
      CENTER,
      RIGHT
   }


   public Scalar lower = new Scalar(62.3, 206, 60.9);
   public Scalar upper = new Scalar(213.9, 255, 255);
   //public int blur = 0;
   public int threshold = 100;

   Mat grey = new Mat();

   static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(40,20);
   static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(145,0);
   static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(240,20);
   static final int REGION_WIDTH = 20;
   static final int REGION_HEIGHT = 35;

   Point region1_pointA = new Point(
         REGION1_TOPLEFT_ANCHOR_POINT.x,
         REGION1_TOPLEFT_ANCHOR_POINT.y);
   Point region1_pointB = new Point(
         REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
         REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
   Point region2_pointA = new Point(
         REGION2_TOPLEFT_ANCHOR_POINT.x,
         REGION2_TOPLEFT_ANCHOR_POINT.y);
   Point region2_pointB = new Point(
         REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
         REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
   Point region3_pointA = new Point(
         REGION3_TOPLEFT_ANCHOR_POINT.x,
         REGION3_TOPLEFT_ANCHOR_POINT.y);
   Point region3_pointB = new Point(
         REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
         REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

   /*
    * A good practice when typing EOCV pipelines is
    * declaring the Mats you will use here at the top
    * of your pipeline, to reuse the same buffers every
    * time. This removes the need to call mat.release()
    * with every Mat you create on the processFrame method,
    * and therefore, reducing the possibility of getting a
    * memory leak and causing the app to crash due to an
    * "Out of Memory" error.
    */
   private Mat ycrcbMat       = new Mat();
   private Mat binaryMat      = new Mat();
   private Mat maskedInputMat = new Mat();

   // Volatile since accessed by OpMode thread w/o synchronization
   private volatile SkystonePosition position = SkystonePosition.LEFT;

   /*
    * Working variables
    */
   Mat region1_Cb, region2_Cb, region3_Cb;
   Mat YCrCb = new Mat();
   Mat Cb = new Mat();
   int avg1, avg2, avg3;

   /*
    * This function takes the RGB frame, converts to YCrCb,
    * and extracts the Cb channel to the 'Cb' variable
    */
   void inputToCb(Mat input)
   {
      Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
      Core.extractChannel(YCrCb, Cb, 2);
   }

   @Override
   public void init(Mat firstFrame)
   {
      /*
       * We need to call this in order to make sure the 'Cb'
       * object is initialized, so that the submats we make
       * will still be linked to it on subsequent frames. (If
       * the object were to only be initialized in processFrame,
       * then the submats would become delinked because the backing
       * buffer would be re-allocated the first time a real frame
       * was crunched)
       */
      inputToCb(firstFrame);

      /*
       * Submats are a persistent reference to a region of the parent
       * buffer. Any changes to the child affect the parent, and the
       * reverse also holds true.
       */
      region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
      region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
      region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
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
      Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

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
      Core.inRange(ycrcbMat, lower, upper, binaryMat);



      /*
       * Release the reusable Mat so that old data doesn't
       * affect the next step in the current processing
       */
      maskedInputMat.release();

      /*
       * Now, with our binary Mat, we perform a "bitwise and"
       * to our input image, meaning that we will perform a mask
       * which will include the pixels from our input Mat which
       * are "255" in our binary Mat (meaning that they're inside
       * the range) and will discard any other pixel outside the
       * range (RGB 0, 0, 0. All discarded pixels will be black)
       */
      Core.bitwise_and(input, input, maskedInputMat, binaryMat);

//      int elementType = Imgproc.MORPH_RECT;
//      int kernelSize = 0;
//      Mat element = Imgproc.getStructuringElement(elementType, new Size(2 * kernelSize + 1, 2 * kernelSize + 1),
//            new Point(kernelSize, kernelSize));
//      Imgproc.dilate(maskedInputMat, maskedInputMat, element);

      Mat cannyOutput = new Mat();
      Imgproc.Canny(maskedInputMat, cannyOutput, threshold, threshold * 2);

      List<MatOfPoint> contours = new ArrayList<>();
      Mat hierarchy = new Mat();
      Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

      MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
      Rect[] boundRect = new Rect[contours.size()];
      Point[] centers = new Point[contours.size()];
      float[][] radius = new float[contours.size()][1];

      for (int i = 0; i < contours.size(); i++) {
         contoursPoly[i] = new MatOfPoint2f();
         Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
         boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
         centers[i] = new Point();
         Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
      }

      Mat drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);

      List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
      for (MatOfPoint2f poly : contoursPoly) {
         contoursPolyList.add(new MatOfPoint(poly.toArray()));
      }

      for (int i = 0; i < contours.size(); i++) {
         Scalar color = new Scalar(200, 111, 222);
         Imgproc.drawContours(drawing, contoursPolyList, i, color);
         //Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
         //Imgproc.circle(drawing, centers[i], (int) radius[i][0], color, 2);
      }

//      telemetry.addData("Analysis", "pipeline.getAnalysis()");
//      telemetry.update();

      return drawing ;
   }


}
