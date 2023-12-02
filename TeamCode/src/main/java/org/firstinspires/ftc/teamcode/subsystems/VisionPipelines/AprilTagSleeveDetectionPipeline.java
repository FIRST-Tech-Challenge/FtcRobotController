package org.firstinspires.ftc.teamcode.subsystems.VisionPipelines;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;

import java.util.ArrayList;

@Config
public class AprilTagSleeveDetectionPipeline extends OpenCvPipeline {
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
    public static int m_stageToShow = 0;
    private long nativeApriltagPtr;

    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat grayMat = new Mat();
    Mat identifiedInput= new Mat();
    Mat cameraMatrix;

    double m_fx = 578.272;
    double m_fy = 578.272;
    double m_cx = 402.145;
    double m_cy = 221.506;

    // UNITS ARE METERS
    double m_tagsize = 0.166;

    int m_parkingSpot = 0;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public AprilTagSleeveDetectionPipeline()
    {
        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public void finalize()
    {
        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
        else
        {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

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

        Imgproc.cvtColor(input, grayMat, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync)
        {
            if(needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grayMat, m_tagsize, m_fx, m_fy, m_cx, m_cy);

        synchronized (detectionsUpdateSync)
        {
            detectionsUpdate = detections;
        }

        input.copyTo(identifiedInput);

        for(AprilTagDetection detection : detections)
        {

            //April tag 1 = parking spot 3
            //April tag 8 = parking spot 1
            //April tag 15 = parking spot 2
            switch(detection.id)
            {
                case 1:
                    m_parkingSpot = 2;
                    break;
                case 8:
                    m_parkingSpot = 0;
                    break;
                case 15:
                    m_parkingSpot = 1;
                    break;
                default:
                    break;
            }
        }

        Imgproc.putText(identifiedInput,
                "Spot:" + Integer.toString(m_parkingSpot + 1),
                new Point(50,150), Imgproc.FONT_HERSHEY_SIMPLEX, 3, new Scalar(255),3);


        switch(m_stageToShow) {
            case 1:
                return grayMat;
            case 2:
                return identifiedInput;
            default:
                return input;
        }

    }

    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, m_fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, m_cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,m_fy);
        cameraMatrix.put(1,2,m_cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    public int getStageToShow()
    {
        return m_stageToShow;
    }

    public int getParkingSpot() {return m_parkingSpot;}
}
