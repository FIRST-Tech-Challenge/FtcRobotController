package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/** Same robot-relative routine on either alliance; no color or gamepad setup. */
@Autonomous(name = "Biobuzz-Team 1 Auto", group = "Quantum Storm", preselectTeleOp = Constants.TELEOP_NAME)
public final class TeamOneAutonomous extends LinearOpMode {
    @Override public void runOpMode(){
        try(AutoStateMachine auto=new AutoStateMachine(this,false)){
            while(opModeInInit()&&!isStopRequested()){auto.updateInitTelemetry();sleep(100);}
            waitForStart();
            if(!isStopRequested())auto.run();
        }catch(java.util.concurrent.CancellationException ignored){
            // STOP during INIT is a normal exit; hardware closes automatically.
        }
    }
}
