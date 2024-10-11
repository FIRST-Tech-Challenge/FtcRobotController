package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Limelight extends LinearOpMode {
    private Limelight3A limelight;
    private int hz = 100;
    private int transInterval = 11;

    public Limelight(HardwareMap hw)
    {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.start();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //rate of data being sent each second

        while(isStopRequested())
        {
            limelight.setPollRateHz(hz);
            limelight.start();

            telemetry.setMsTransmissionInterval(transInterval);
            limelight.pipelineSwitch(0);
            limelight.start();
        }

    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }
}
