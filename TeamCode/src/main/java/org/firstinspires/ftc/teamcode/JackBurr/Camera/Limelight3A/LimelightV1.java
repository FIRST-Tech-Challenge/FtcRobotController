package org.firstinspires.ftc.teamcode.JackBurr.Camera.Limelight3A;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class LimelightV1 {
    public HardwareMap hardwareMap;
    public MultipleTelemetry telemetry;
    public Limelight3A limelight;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.limelight = this.hardwareMap.get(Limelight3A.class, "limelight");
    }

    public Limelight3A getLimelight(){
        return limelight;
    }

    public int getVersion(){
        return limelight.getVersion();
    }

    public void startStreaming(){
        limelight.start();
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
        if(limelight.getLatestResult() != null) {
            return limelight.getLatestResult();
        }
        else {
            return null;
        }
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

    public void restart(int pipelineToSet){
        limelight.close();
        startStreaming();
        setPipeline(pipelineToSet);
    }

    public double getFps(){
        return getStatus().getFps();
    }
    public void pause(){
        limelight.pause();
    }
    public List<LLResultTypes.ColorResult> getColorResults(){
        List<LLResultTypes.ColorResult> list = new ArrayList<>();
        if (limelight.getLatestResult() != null) {
            return limelight.getLatestResult().getColorResults();
        }
        else {
            return list;
        }
    }
    public LLResultTypes.ColorResult getColorResultFromList(List<LLResultTypes.ColorResult> list, int index){
        return list.get(index);
    }
    public double getAngle(){
        if(!getColorResults().isEmpty()) {
            List<List<Double>> corners = getColorResults().get(getColorResults().size()-1).getTargetCorners();
            if (corners.size() >= 4) {
                double x1 = corners.get(0).get(0); // Top-left corner X
                double y1 = corners.get(0).get(1); // Top-left corner Y
                double x2 = corners.get(1).get(0); // Top-right corner X
                double y2 = corners.get(1).get(1); // Top-right corner Y
                double angleRadians = Math.atan2(y2 - y1, x2 - x1);
                return Math.toDegrees(angleRadians);
            }
            else {
                telemetry.addLine("Not enough results.");
                return 0.0;
            }

        }
        else {
            telemetry.addLine("No results.");
            return 0.0;
        }
    }
}
