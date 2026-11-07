package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.teamPedroPathing.PedroPathConfiguration;

import java.util.List;


//why is this here I deleted it
@Disabled
@TeleOp
public class StarterOpMode extends OpMode{
    private RobotHardware robotHardware;



    private Follower follower;
    private Limelight3A limelight;
    private boolean bWasPressed;
    private int gameNum;
    @Override
    public void init() {

        PedroPathConfiguration pedroPathConfiguration = new PedroPathConfiguration( this );
      follower = pedroPathConfiguration.getFollower();
        robotHardware = new RobotHardware(this);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();
        bWasPressed = false;
    }
     public void init_loop() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult.isValid() ) {
            telemetry.addLine("results are valid");
           List<LLResultTypes.FiducialResult> results = llResult.getFiducialResults();
           if (!results.isEmpty()) {
               telemetry.addLine("results are not empty ");
               gameNum = results.get(0).getFiducialId();
           }
        }
        telemetry.addData("tagNumber is", gameNum);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()){
            //robotHardware.liftUp();
        }
        if (gamepad1.dpadDownWasPressed()){
            //robotHardware.liftDown();
        }

        follower.update();
        follower.manual( -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        if (gamepad1.x){
            robotHardware.intakeOn();
        } else{
           robotHardware.intakeOff();
        }
        if (gamepad1.bWasPressed()) {
            if (bWasPressed) {
                    //robotHardware.vertTube();
                    bWasPressed = false;
            } else {
                //robotHardware.tiltTube();
                bWasPressed = true;
            }
        }
        if (gamepad1.y){
            robotHardware.outtakeOn();
        }else{
          robotHardware.intakeOff();
        }

    }


}

