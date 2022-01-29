package com.SCHSRobotics.HAL9001.system.robot.subsystems.calib;

import android.util.Log;

import com.SCHSRobotics.HAL9001.system.robot.Camera;
import com.SCHSRobotics.HAL9001.system.robot.HALPipeline;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.VisionSubSystem;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;

import org.jetbrains.annotations.NotNull;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * A class for calibrating the phone camera in real-ish (surreal?) time.
 *
 * @author Cole Savage, Level Up
 * @since 1.0.0
 * @version 1.0.0
 *
 * Creation Date: 8/24/19
 */
@SuppressWarnings("unused")
public class CameraCalib extends VisionSubSystem {

    //The customizable gamepad holding all the controls for the calibration class.
    private CustomizableGamepad inputs;
    //The names all the gamepad's buttons.
    private static final String CAPTURE = "capture", DELETE_CAPTURE = "delete capture", CALIBRATE = "calibrate";
    //A flag value used to detect if any of the buttons have been pushed.
    private boolean flag;
    //Boolean values specifying the state of the camera calibration.
    private boolean calibrationBegun = false, calibrated = false;
    //Lists of points on the calibration pattern, both expected and detected.
    private List<Mat> refPoints, capturePoints;
    //Camera matrix and the distortion coefficients.
    private Mat intrinsic, distCoeffs;
    //Rotation vectors and translation vectors to be applied to correct the distortion.
    private List<Mat> rvecs, tvecs;
    //The reference coordinates.
    private MatOfPoint3f refCoords;
    //The width and height of the camera frame.
    private int width, height;
    //The size of the calibration pattern.
    private Size size;
    //The reprojection error.
    private double reprojError;

    /**
     * Constructor for CameraCalib.
     *
     * @param robot The robot using this subsystem.
     * @param chessboardSize The size of the chessboard pattern.
     */
    public CameraCalib(@NotNull Robot robot, @NotNull Size chessboardSize) {
        super(robot);

        inputs = new CustomizableGamepad(robot);
        inputs.addButton(CAPTURE, new Button(1, Button.BooleanInputs.x));
        inputs.addButton(DELETE_CAPTURE, new Button(1, Button.BooleanInputs.b));
        inputs.addButton(CALIBRATE, new Button(1, Button.BooleanInputs.y));

        refPoints = new ArrayList<>();
        capturePoints = new ArrayList<>();

        size = chessboardSize;

        refCoords = new MatOfPoint3f();
        double squareSize = 1;
        Point3[] vp = new Point3[(int) (size.width * size.height)];
        int cnt = 0;
        for (int i = 0; i < size.width; ++i) {
            for (int j = 0; j < size.height; ++j, cnt++) {
                vp[cnt] = new Point3(j * squareSize, i * squareSize, 0.0d);
            }
        }
        refCoords.fromArray(vp);

        intrinsic = Mat.eye(3,3, CvType.CV_64F);
        distCoeffs = Mat.zeros(8,1, CvType.CV_64F);

        rvecs = new ArrayList<>();
        tvecs = new ArrayList<>();

        flag = true;
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }

    @Override
    public void handle() {

        if(!(boolean) inputs.getInput(CAPTURE) && !(boolean) inputs.getInput(DELETE_CAPTURE) && (boolean) inputs.getInput(CALIBRATE) && !calibrationBegun && capturePoints.size() > 10 && flag) {
            calibrationBegun = true;

            Thread calibrate = new Thread() {
                @Override
                public void run() {
                    reprojError = Calib3d.calibrateCamera(refPoints, capturePoints, new Size(width / 2, height / 2), intrinsic, distCoeffs, rvecs, tvecs);
                    calibrated = true;
                }
            };

            calibrate.start();

            flag = false;
        }
        else if(!(boolean) inputs.getInput(CAPTURE) && !(boolean) inputs.getInput(DELETE_CAPTURE) && !(boolean) inputs.getInput(CALIBRATE) && !flag) {
            flag = true;
        }
    }

    @Override
    public void stop() {
        for (int i = 0; i < capturePoints.size(); i++) {
            capturePoints.get(i).release();
            refPoints.get(i).release();
        }
        refCoords.release();
    }

    @Override
    protected HALPipeline[] getPipelines() {
        return new HALPipeline[]{new CameraCalibPipeline()};
    }

    @Camera(id = Robot.ALL_CAMERAS_ID)
    public class CameraCalibPipeline extends HALPipeline {
        @Override
        public boolean useViewport() {
            return robot.isStarted();
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat gray = new Mat();

            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);
            Imgproc.resize(gray, gray, new Size(width / 2, height / 2));
            input.release();

            if (!calibrationBegun) {
                MatOfPoint2f corners = new MatOfPoint2f();
                boolean found = Calib3d.findChessboardCorners(gray, size, corners, Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_NORMALIZE_IMAGE | Calib3d.CALIB_CB_FAST_CHECK);
                if (found) {
                    TermCriteria term = new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 40, 0.001);

                    Imgproc.cornerSubPix(gray, corners, new Size(5, 5), new Size(-1, -1), term);

                    Imgproc.cvtColor(gray, gray, Imgproc.COLOR_GRAY2RGB);

                    Calib3d.drawChessboardCorners(gray, size, corners, true);

                    if ((boolean) inputs.getInput(CAPTURE) && flag) {
                        refPoints.add(refCoords);
                        capturePoints.add(corners);
                        flag = false;
                    } else if ((boolean) inputs.getInput(DELETE_CAPTURE) && refPoints.size() > 0 && flag) {
                        refPoints.remove(refPoints.size() - 1);
                        capturePoints.remove(capturePoints.size() - 1);
                        flag = false;
                    } else if (!(boolean) inputs.getInput(CAPTURE) && !(boolean) inputs.getInput(DELETE_CAPTURE) && !flag) {
                        flag = true;
                    }

                    Log.wtf("test", "" + refPoints.size());
                }
            } else if (calibrated) {
                Log.wtf("done", intrinsic.dump());
                Log.wtf("done", "" + reprojError);

                Mat undistorted = new Mat();

                Calib3d.undistort(gray, undistorted, intrinsic, distCoeffs);

                Imgproc.resize(undistorted, undistorted, new Size(width, height));

                return undistorted;

            }

            Imgproc.resize(gray, gray, new Size(width, height));

            return gray;
        }
    }
}