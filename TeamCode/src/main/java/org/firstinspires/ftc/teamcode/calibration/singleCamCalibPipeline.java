package org.firstinspires.ftc.teamcode.calibration;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class singleCamCalibPipeline extends OpenCvPipeline {
    public boolean calibrate = false;
    // the saved chessboard image
    private Mat savedImage;

    private List<Mat> imagePoints = new ArrayList<>();
    private List<Mat> objectPoints = new ArrayList<>();
    private MatOfPoint3f obj = new MatOfPoint3f();
    private MatOfPoint2f imageCorners = new MatOfPoint2f();
    private int boardsNumber = 10;
    private int numCornersHor = 6;
    private int numCornersVer = 6;
    private int successes = 0;
    private Mat intrinsic = new Mat(3,3, CvType.CV_8UC1);
    private Mat distCoeffs = new Mat();
    public boolean isCalibrated = false;

    @Override
    public Mat processFrame(Mat mat) {
        if(calibrate && !isCalibrated && !mat.empty()) {
            Mat grayImage = new Mat();

            // I would perform this operation only before starting the calibration
            // process
            if (this.successes < this.boardsNumber)
            {
                // convert the frame in gray scale
                Imgproc.cvtColor(mat, grayImage, Imgproc.COLOR_BGR2GRAY);
                // the size of the chessboard
                Size boardSize = new Size(this.numCornersVer, this.numCornersHor);
                // look for the inner chessboard corners
                boolean found = Calib3d.findChessboardCorners(grayImage, boardSize, imageCorners,
                        Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK);
                // all the required corners have been found...
                if (found)
                {
                    // optimization
                    TermCriteria term = new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.1);
                    Imgproc.cornerSubPix(grayImage, imageCorners, new Size(11, 11), new Size(-1, -1), term);
                    // save the current frame for further elaborations
                    grayImage.copyTo(this.savedImage);
                    if (this.successes < this.boardsNumber)
                    {
                        // save all the needed values
                        this.imagePoints.add(imageCorners);
                        imageCorners = new MatOfPoint2f();
                        this.objectPoints.add(obj);
                        this.successes++;
                    }
                }
            }
        }
        if (this.successes == this.boardsNumber)
        {
            this.calibrate();
        }
        return null;
    }
    public void calibrate() {
        // init needed variables according to OpenCV docs
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();
        intrinsic.put(0, 0, 1);
        intrinsic.put(1, 1, 1);
        // calibrate!
        Calib3d.calibrateCamera(objectPoints, imagePoints, savedImage.size(), intrinsic, distCoeffs, rvecs, tvecs);
        this.isCalibrated = true;

    }
}
