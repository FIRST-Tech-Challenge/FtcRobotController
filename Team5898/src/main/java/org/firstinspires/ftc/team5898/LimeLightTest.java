package org.firstinspires.ftc.team5898;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class LimeLightTest extends OpMode {

    private Limelight3A limelight3A;

    @Override
    public void init()
    {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0); // 0 is blue & 1 is red

    }

    @Override
    public void start()
    {
        limelight3A.start();
    }

    @Override
    public void loop()
    {
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid())
        {
            telemetry.addData("Target X offset", llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target area offset", llResult.getTa());
            telemetry.update();
        }
    }
}
