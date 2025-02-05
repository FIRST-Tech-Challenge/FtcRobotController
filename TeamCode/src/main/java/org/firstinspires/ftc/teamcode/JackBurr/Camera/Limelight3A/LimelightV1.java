package org.firstinspires.ftc.teamcode.JackBurr.Camera.Limelight3A;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import java.util.List;


public class LimelightV1 {
    public HardwareMap hardwareMap;
    public Limelight3A limelight;

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.limelight = this.hardwareMap.get(Limelight3A.class, "limelight");
    }

    public Limelight3A getLimelight(){
        return limelight;
    }

    public int getVersion(){
        return limelight.getVersion();
    }

    public void close(){
        limelight.close();
    }

    public boolean isConnected(){
        return limelight.isConnected();
    }

    public boolean isRunning(){
        return limelight.isRunning();
    }

    public boolean setPipeline(int index){
        return limelight.pipelineSwitch(index);
    }

    public LLResult getLatestResult(){
        return limelight.getLatestResult();
    }

    public LLStatus getStatus(){
        return limelight.getStatus();
    }

    public boolean captureSnapshot(String snapshotName){
        return limelight.captureSnapshot(snapshotName);
    }

    public boolean deleteSnapshot(String snapshotName){
        return limelight.deleteSnapshot(snapshotName);
    }

    public double getFps(){
        return getStatus().getFps();
    }
    public void pause(){
        limelight.pause();
    }
    public List<LLResultTypes.ColorResult> getColorResults(){
        return limelight.getLatestResult().getColorResults();
    }
    public LLResultTypes.ColorResult getColorResultFromList(List<LLResultTypes.ColorResult> list, int index){
        return list.get(index);
    }
    public double getAngle(){
        double angle = -1;
        if(!getColorResults().isEmpty()) {
            List<LLResultTypes.ColorResult> list = getColorResults();
            LLResultTypes.ColorResult result = getColorResultFromList(list, list.size() -1);
            List<List<Double>> points = result.getTargetCorners();
            if(!points.isEmpty()){
                for (List<Double> pointSet : points) {
                    if (pointSet.size() % 2 != 0) {
                        System.out.println("Invalid point list size.");
                        continue;
                    }

                    Point[] contourArray = new Point[pointSet.size() / 2];
                    for (int i = 0; i < pointSet.size(); i += 2) {
                        contourArray[i / 2] = new Point(pointSet.get(i), pointSet.get(i + 1));
                    }

                    MatOfPoint2f contourMat = new MatOfPoint2f(contourArray);
                    RotatedRect rect = Imgproc.minAreaRect(contourMat);
                    angle = rect.angle;
                    if (rect.size.width < rect.size.height) {
                        angle = rect.angle - 90;
                    }
                    else {
                        angle = rect.angle;
                    }
                    // Example: Printing or using the rectangle properties
                    System.out.println("Rect Center: " + rect.center);
                    System.out.println("Rect Angle: " + rect.angle);
                    return rect.angle;
                }
            }
        }
        else {
            return -1;
        }
        return angle;
    }
}
