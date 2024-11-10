package org.firstinspires.ftc.teamcode.bots;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightBot extends GyroBot {

    public LLResult result = null;
    public Pose3D botpose = null;
    public Limelight3A limelight;

    @Override

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);

        limelight.start();


    }

    public LimelightBot(LinearOpMode opMode) {
        super(opMode);
    }


    public double[] detectOne() {
        double[] values = new double[3];
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                double xDegree = result.getTx();
                double yDegree = result.getTy();
                double angle = result.getPythonOutput()[8];

                double xResult = xDegree;
                double yResult = yDegree;
                values[0] = xResult;
                values[1] = yResult;
                values[2] = angle;
            }
        }

        return values;

    }

    public void switchPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
}


