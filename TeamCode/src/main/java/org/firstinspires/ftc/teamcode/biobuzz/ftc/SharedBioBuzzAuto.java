package org.firstinspires.ftc.teamcode.biobuzz.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Sensors;

/** FTC lifecycle only. Hardware initializes and calibrates while the robot is still in INIT. */
public abstract class SharedBioBuzzAuto extends LinearOpMode {
    protected abstract int alliance();
    protected FtcHardwareConfig hardwareConfig() { return new FtcHardwareConfig(); }
    @Override public void runOpMode() {
        int alliance=alliance();FtcHardwareConfig config=hardwareConfig();
        try(FtcRobotIO io=new FtcRobotIO(hardwareMap,config,alliance,this::isStopRequested)){
            AutoStateMachine auto=new AutoStateMachine(io,config.field,config.useVision,AutoStateMachine.Routine.FULL,config.movementOnly);
            try{
                while(!isStarted()&&!isStopRequested()){
                    Sensors s=io.readSensors();
                    telemetry.addData("Alliance",alliance==0?"Red":"Blue");
                    telemetry.addData("Vision",config.useVision?"Limelight required":"Camera-free: fixed poses, tilt unverified");
                    telemetry.addData("Mechanisms",config.movementOnly?"DISABLED: simulator-timed movement only":"Enabled");
                    telemetry.addData("Odometry / camera / balls",s.odometryValid+" / "+s.cameraConnected+" / "+s.ballCount);
                    telemetry.addLine("Place robot at the calibrated start and keep still.");telemetry.update();idle();
                }
                if(isStopRequested())return;
                auto.start(alliance,config.shooterPower);
                while(opModeIsActive()&&auto.isRunning()){auto.tick();telemetry.addData("Autonomous",auto.status());telemetry.update();idle();}
            }finally{auto.stop();}
        }catch(java.util.concurrent.CancellationException e){/* STOP is a normal exit. */}
    }
}
