package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
<<<<<<< Updated upstream
=======
import com.arcrobotics.ftclib.vision.AprilTag2dPipeline;
>>>>>>> Stashed changes
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
<<<<<<< Updated upstream
import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.RedDetectPipeline;
import org.firstinspires.ftc.teamcode.subsystems.visionpipelines.BlueDetectPipeline;

=======
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robotContainer.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.BlueDetectPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.RedDetectPipeline;


import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;
>>>>>>> Stashed changes
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

<<<<<<< Updated upstream
=======
import java.util.ArrayList;

>>>>>>> Stashed changes
public class VisionSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    OpenCvWebcam m_webcam;
<<<<<<< Updated upstream
    RedDetectPipeline m_redImagePipeline;
=======
    BlueDetectPipeline m_redImagePipeline;
    //AprilTagDetectionPipeline m_aprilTagDetection;
>>>>>>> Stashed changes
    Boolean m_showStage = Boolean.TRUE;
    Pose2d deposit1;
    Pose2d deposit2;
    Pose2d deposit3;
    public int pathNum = -1;
<<<<<<< Updated upstream

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_redImagePipeline = new RedDetectPipeline();
=======
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    static final double FEET_PER_METER = 3.28084;
    public ArrayList<AprilTagDetection> detections;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_redImagePipeline = new BlueDetectPipeline();

>>>>>>> Stashed changes

        int cameraMonitorViewId = m_hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", m_hardwareMap.appContext.getPackageName());
        m_webcam = OpenCvCameraFactory.getInstance().createWebcam(m_hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        m_webcam.setPipeline(m_redImagePipeline);
        m_webcam.setMillisecondsPermissionTimeout(2500);
        m_webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                           @Override
                                           public void onOpened() {
                                               m_webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                                           }

                                           @Override
                                           public void onError(int errorCode) {
                                               //if can't open the webcam do something here
                                           }
                                       }
        );

        //output the OpenCV processed image from the webcam to the FTCDashboard
        FtcDashboard.getInstance().startCameraStream(m_webcam, 30);
    }
<<<<<<< Updated upstream
    public int getLocation(){
        return m_redImagePipeline.getLocation();
    }

    @Override
    public void periodic()

    {
            pathNum = m_redImagePipeline.getLocation();
            m_telemetry.addData("Location:", m_redImagePipeline.getLocation());
            m_telemetry.update();

    }

    public void disablePipeline()
    {
        m_webcam.stopStreaming();
        m_showStage = Boolean.FALSE;
    }





}
=======
//
    public int getLocation() {
        return m_redImagePipeline.getLocation();
    }



//    detection.ftcPose.range = Math.hypot(detection.ftcPose.x, detection.ftcPose.y);
//    detection.ftcPose.bearing = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-detection.ftcPose.x, detection.ftcPose.y));
//    detection.ftcPose.elevation = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(detection.ftcPose.z, detection.ftcPose.y));



    public void setMode(String mode) {

        if (mode == "BLUE") {
            m_redImagePipeline.H_start = Constants.H_START_BLUE;
            m_redImagePipeline.H_end = Constants.H_END_BLUE;
        } else if (mode == "RED") {
            m_redImagePipeline.H_start = Constants.H_START_RED;
            m_redImagePipeline.H_end = Constants.H_END_RED;

        }
    }

    public void disablePipeline() {
        m_webcam.stopStreaming();
        m_showStage = Boolean.FALSE;
    }
    @Override
    public void periodic() {
        if(m_showStage) {
            m_telemetry.addData("Detected Spot:", m_redImagePipeline.getLocation());
            m_telemetry.update();
        }
    }


}


>>>>>>> Stashed changes
