package org.firstinspires.ftc.team5898;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

// This is for location tracking on the field using AprilTags and the Limelight 3A (MegaTag V2)
@Autonomous(name="LimeLight Test")
@Config
public class LimeLightAprilTagTrack extends OpMode {
    FtcDashboard dashboard;
    Limelight3A limelight3A;
    public static int pipeLineSelect = 0;
    IMU imu;


    @Override
    public void init()
    {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        imu = hardwareMap.get(IMU.class, "imu");
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(pipeLineSelect);

    }

    @Override
    public void start()
    {
        limelight3A.start();
        if (limelight3A instanceof CameraStreamSource) {
            try {
                dashboard.startCameraStream((CameraStreamSource) limelight3A, 60);
            } catch (Throwable ignore) {}
        }
    }

    @Override
    public void loop()
    {
        limelight3A.pipelineSwitch(pipeLineSelect);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Pipeline", pipeLineSelect);

        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid())
        {
            double tx = llResult.getTx();
            double ty = llResult.getTy();
            double ta = llResult.getTa();

            telemetry.addData("Target X offset", tx);
            telemetry.addData("Target Y offset", ty);
            telemetry.addData("Target area", ta);

            packet.put("tx", tx);
            packet.put("ty", ty);
            packet.put("ta", ta);
            packet.put("status", "target detected");
        }
        else
        {
            telemetry.addLine("No valid target");
            packet.put("status", "no target");
        }

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
}
