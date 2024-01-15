package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static android.content.ContentValues.TAG;
import static org.opencv.core.Core.minMaxLoc;

public class QRdetector {
    
    private OpenCvWebcam webcam;
    private QRCodeDetector detector;
    private Mat detectedLocation;
    double[] centerPoint = {0,0};
    double maxVal;
    long exposureTime = 100;
    ExposureControl exposureControl;
    
    QRdetector(RobotHardware H) {
        
        detector = new QRCodeDetector();
        detectedLocation = new Mat();
        
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(H.webcam, H.monitorViewId);
        webcam.setPipeline(new OpenCvPipeline() {
        
            final int numAveragePoints = 10;
            double[][] lastPoints = new double[numAveragePoints][2];
            int storeIterator = 0;
            List<MatOfPoint> list = new ArrayList<>();
            Mat greyscale = new Mat();
        
            @Override
            public Mat processFrame(Mat input) {
                greyscale.release();
                Imgproc.cvtColor(input, greyscale, Imgproc.COLOR_RGB2GRAY);
                maxVal = minMaxLoc(greyscale).maxVal;
                //exposureTime -= 1;
                //exposureControl.setExposure(exposureTime, TimeUnit.MILLISECONDS);
                //webcam.
                input.convertTo(input, -1,255/maxVal,0);
                
                double[][] points = new double[5][2]; // 1, 2, 3, 4, center
                
                // tell the detector to try to find something
                detector.detect(input, detectedLocation);
                
                // if something is found do this
                if (!detectedLocation.empty()) {
                
                    // clear the last bounding box points
                    list.clear();
                
                    // average all bounding box points to get the center point
                    for (int i = 0; i < 4; i++) {
                        points[i] = detectedLocation.get(0, i);
                        points[4][0] += points[i][0];
                        points[4][1] += points[i][1];
                    }
                    points[4][0] /= 4;
                    points[4][1] /= 4;
                
                    // store the last numAveragePoints (10) points
                    lastPoints[storeIterator] = points[4];
                    storeIterator ++;
                    if (storeIterator >= numAveragePoints) storeIterator = 0;
                
                    // find the average center point
                    centerPoint[0] = 0;
                    centerPoint[1] = 0;
                    for (int i = 0; i < numAveragePoints; i ++) {
                        centerPoint[0] += lastPoints[i][0];  // current point is included in lastPoints
                        centerPoint[1] += lastPoints[i][1];
                    }
                    centerPoint[0] /= numAveragePoints;
                    centerPoint[1] /= numAveragePoints;
                
                    list.add( new MatOfPoint (
                            new Point(points[0][0], points[0][1]), new Point(points[1][0], points[1][1]),
                            new Point(points[2][0], points[2][1]), new Point(points[3][0], points[3][1])));
                
                    detectedLocation.release();
                
                }
            
                // draw what is detected and average center from last n detections
                Imgproc.polylines(input, list, true, new Scalar(255,127,0),5);
                Imgproc.circle(input, new Point(points[4][0],points[4][1]),3 , new Scalar(255,127,0),6);
                Imgproc.circle(input, new Point(centerPoint[0],centerPoint[1]),3 , new Scalar(0,127,255),6);
                
                
                return input;
            }
        });
    
        webcam.showFpsMeterOnViewport(true);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        
            @Override
            public void onOpened() {
    
                exposureControl = webcam.getExposureControl();
                exposureControl.setMode(ExposureControl.Mode.AperturePriority);
                //exposureControl.setExposure(1, TimeUnit.MILLISECONDS);
                webcam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            
            }
        
            @Override
            public void onError(int errorCode) {
    
                Log.e(TAG, "Could not open camera, error code: " + errorCode);
            
            }
        });
        
    }
    
    public void shutDown() {
        webcam.stopStreaming();
    }
    
    public double[] getCenterPoint() {
        
        return centerPoint;
    }
    
    public float getFps() {
        
        return webcam.getFps();
        
    }
    
}
