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
    private Limelight3A limelight;

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
    public double horizontalDistance(){
        result = limelight.getLatestResult();
        return result.getTx();

    }

    public double verticalDistance(){
        result = limelight.getLatestResult();
        return result.getTy();
    }
   public double[] detectOne(boolean button){
       LLResult result = limelight.getLatestResult();
       if(result != null){
           if(result.isValid()){
               Pose3D botpose = result.getBotpose();
               telemetry.addData("tx", result.getTx());
               telemetry.addData("ty", result.getTy());
               telemetry.addData("Botpose", botpose.toString());
           }
       }
       double[] temp = new double[3];
       return temp;
    }

}