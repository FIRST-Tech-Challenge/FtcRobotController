package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/** Contingency Team 1 mode: two Garden passes, then the Team 2 wall shot if a tag is live. */
@Autonomous(name = "Biobuzz-Team 1 Backup Auto", group = "Quantum Storm", preselectTeleOp = Constants.TELEOP_NAME)
public final class TeamOneBackupAutonomous extends LinearOpMode {
    @Override public void runOpMode(){
        try(AutoStateMachine auto=new AutoStateMachine(this,false,true)){
            while(opModeInInit()&&!isStopRequested()){auto.updateInitTelemetry();sleep(100);}
            waitForStart();
            if(!isStopRequested())auto.run();
        }catch(java.util.concurrent.CancellationException ignored){
            // STOP during INIT is a normal exit; hardware closes automatically.
        }
    }
}
