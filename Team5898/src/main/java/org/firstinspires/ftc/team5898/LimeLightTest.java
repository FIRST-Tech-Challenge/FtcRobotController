package org.firstinspires.ftc.team5898;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

@Autonomous
@Config
public class LimeLightTest extends OpMode {
    private FtcDashboard dashboard;
    private Limelight3A limelight3A;
    public static int pipeLineSelect = 0;
    @Override
    public void init()
    {
        //FTC Dashboard Init
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Limelight Init
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(pipeLineSelect); // 0 is blue & 1 is red & 2 is yellow
    }

    @Override
    public void start()
    {
        limelight3A.start();
        dashboard.startCameraStream((CameraStreamSource) limelight3A, 60.0);
    }

    @Override
    public void loop()
    {
        //Sync Telemetry to Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Pipeline", pipeLineSelect);
        dashboard.sendTelemetryPacket(packet);



        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid())
        {
            telemetry.addData("Target X offset", llResult.getTx());
            packet.put("Target X offset", llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target area offset", llResult.getTa());
            telemetry.update();
        }


    }
}
