package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Blahblahblah",group="Sensor")
public class LimelightTry extends OpMode{
    private Limelight3A limelight;
    public void init(){
        limelight=hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }
    public void loop(){
        LLResult result = limelight.getLatestResult();
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            double firstOutput = pythonOutputs[0];
        }
    }
}
