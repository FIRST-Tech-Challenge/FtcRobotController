package org.firstinspires.ftc.team13580.Autonomous;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team13580.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="Camara", group="Robot")
public class CamaraDetect extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);
    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init();
        waitForStart();;

        AprilTagProcessor tag= new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(1130.80338323,1130.80338323, 1280.21111078, 368.731101737 )
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        while (!isStopRequested() && opModeIsActive() ){
            if (tag.getDetections().size() > 0){
                AprilTagDetection detect= tag.getDetections().get(0);
                telemetry.addData ("x", detect.ftcPose.x);
                telemetry.addData ("y", detect.ftcPose.y);
                telemetry.addData ("z", detect.ftcPose.z);
                telemetry.addData ("roll", detect.ftcPose.roll);
                telemetry.addData("pitch", detect.ftcPose.pitch);
                telemetry.addData("yaw", detect.ftcPose.yaw);
            }
            telemetry.update();
        }
    }
}
