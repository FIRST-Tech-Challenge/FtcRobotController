package org.firstinspires.ftc.teamcode.subsystems.VisionPipelines;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.config.Config;

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

@Config
public class SleeveDetectionPipeline extends OpenCvPipeline {
    public static int m_stageToShow = 0;

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
// 80 to 50 : green
    // 120 to 90: cyan
    // 180 to 160: purple
    Mat mask = new Mat();
    Mat erodedMask = new Mat();
    Mat input_hsv = new Mat();
    Mat input_bgr = new Mat();
    Mat cropped = new Mat();
    Mat input_withAngle = new Mat();
    Scalar sumValue = new Scalar(0);
    float totalPixs = 0.0f;
    float[] sumValNorm = new float[3];
    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    List<MatOfPoint> polys = new ArrayList<MatOfPoint>();
    Mat m_hierarchy = new Mat();
    Mat input_withContours = new Mat();
    Mat input_withSides = new Mat();
    Mat input_withPoints = new Mat();
    Mat erodedMaskForDisplay = new Mat();
    //Mat input_withPollyApprox = new Mat();
    int maxContourIdx = 0;
    double maxArea = 0.0;
    double centerX = 0.0;
    double centerY = 0.0;
    double pixToAngleFactor = 0.0703125;
    double angleX = 0.0;
    int imageSizeX = 640;
    int imageSizeY = 480;
    int maxAreaIndex = 0;

    int m_parkingSpot = 0;

//    public static int H_start = 50;
//    public static int H_end = 80;


    //green {50 to 80}
    //cyan  {100 to 110}
    //magenta {160 to 172}
    public static int greenStart = 50;//was70
    public static int greenEnd = 80;
    public static int cyanStart = 100;//was100
    public static int cyanEnd = 110;//was110
    public static int magentaStart = 160;
    public static int magentaEnd = 172;
    public int[] HstartArray = {magentaStart, greenStart, cyanStart};
    public int[] HendArray = {magentaEnd, greenEnd, cyanEnd};
    double[] ContourArea = new double[3];

    public static int erodedMaskLevel = 0;

    public static int rowCutoff = 170;

    MatOfPoint poly = new MatOfPoint();
    MatOfPoint2f poly2f = new MatOfPoint2f();
    MatOfPoint2f dst2f = new MatOfPoint2f();
    MatOfPoint dst = new MatOfPoint();

    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5), new Point(-1,-1));

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */
//            USE LOWERlim ~60-70, UPPERLIM ~80-90 for green detection
        Imgproc.cvtColor(input, input_bgr,Imgproc.COLOR_RGBA2BGR); //EasyOpenCV return images in RGBA format
        Imgproc.cvtColor(input_bgr, input_hsv, Imgproc.COLOR_BGR2HSV); // We convert them to BGR since only BGR (or RGB) conversions to HSV exist

        HstartArray[0] = magentaStart;
        HstartArray[1] = greenStart;
        HstartArray[2] = cyanStart;

        HendArray[0] = magentaEnd;
        HendArray[1] = greenEnd;
        HendArray[2] = cyanEnd;

        for(int n = 0; n < 3; n++) {
            Core.inRange(input_hsv,
//                    new Scalar(H_start, 50, 50),
//                    new Scalar(H_end, 255, 255),
                    new Scalar(HstartArray[n], 50, 50),
                    new Scalar(HendArray[n], 255, 255),
                    mask);

            for(int m = 0; m < rowCutoff; m++)
            {
                mask.row(m).setTo(new Scalar(0));
            }

            //Erode/Dialate the image to get rid of small pixels
            Imgproc.erode(mask, erodedMask, kernel);

            //Draw contours around detected pixels
            contours.clear();
            Imgproc.findContours(erodedMask, contours, m_hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE );

            if(erodedMaskLevel == n)
            {
                erodedMask.copyTo(erodedMaskForDisplay);
            }

            ContourArea[n] = 0.0;
            maxContourIdx = 0;
            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
            {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                if (ContourArea[n] < contourArea)
                {
                    ContourArea[n] = contourArea;
                    maxContourIdx = contourIdx;
                }
            }
        }

        maxArea = 0.0;
        maxAreaIndex = 0;
        for (int areaIndex=0; areaIndex<3; areaIndex++)
        {
            if (ContourArea[areaIndex] > maxArea)
            {
                maxArea = ContourArea[areaIndex];
                maxAreaIndex = areaIndex;
            }
        }

        input.copyTo(input_withContours);
        Imgproc.putText(input_withContours,
                "Spot:" + Integer.toString(maxAreaIndex + 1),
                new Point(50,150), Imgproc.FONT_HERSHEY_SIMPLEX, 3, new Scalar(255),3);

//        if (contours.size() != 0 )
//        {
//            poly = contours.get(maxContourIdx);
//            poly.convertTo(poly2f, CvType.CV_32F);
//
//            Imgproc.approxPolyDP(poly2f, dst2f, 3.00, true);
//            dst2f.convertTo(dst, CvType.CV_32S);
//            polys.clear();
//            polys.add(dst);
//            Imgproc.drawContours(input_withContours, polys, 0, new Scalar(255), 5);
//        //input.copyTo(input_withPollyApprox);
//        }

        //Imgproc.putText(input_withPollyApprox, "Points" + dst.toArray(), input_withContours, 10, );


//        input_withContours.copyTo(input_withSides);

//       Imgproc.putText(input_withContours,
//               "#pts: " + dst.toArray().length,
//               new Point(40,70), Imgproc.FONT_HERSHEY_SIMPLEX,
//               2, new Scalar(255));

        //park in the spot that has the biggest index
        m_parkingSpot = maxAreaIndex;

        switch(m_stageToShow) {
            case 1:
                return mask;
            case 2:
                return erodedMaskForDisplay;
            case 3:
                return input_withContours;
            default:
                return input;
        }

//45.22 degrees horiontal feild of view
//55 degrees diagonal FOV
        //conversion factor = 45/640
    }
    public int getStageToShow()
    {
        return m_stageToShow;
    }

    public int getParkingSpot() {return m_parkingSpot;}
}
