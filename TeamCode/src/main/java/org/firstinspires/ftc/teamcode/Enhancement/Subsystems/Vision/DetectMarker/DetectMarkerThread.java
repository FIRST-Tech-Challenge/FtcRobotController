package org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Vision.DetectMarker;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Enhancement.Robot;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class DetectMarkerThread implements Runnable {

    Robot robot;
    HardwareMap hardwareMap;
    OpenCvInternalCamera robotCamera;
    MarkerLocation markerLocation = MarkerLocation.NOT_FOUND;

    public DetectMarkerThread(Robot robot, OpenCvInternalCamera camera) {
        this.robot = robot;
        this.hardwareMap = robot.getOpMode().hardwareMap;
        this.robotCamera = camera;

    }


    @Override
    public void run() {
        DetectMarker detectMarker = new DetectMarker(this.hardwareMap, robotCamera, robot.getQuickTelemetry());
        markerLocation = detectMarker.DetectMarkerRun();
    }

    public MarkerLocation getMarkerLocation() {
        return this.markerLocation;
    }
}
