package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.skills.RingDetector;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;

public abstract class AutoBase extends LinearOpMode {
    protected UltimateBot robot = new UltimateBot();
    protected ElapsedTime runtime = new ElapsedTime();

    protected void runAutoMode(){
        initRobot();
        try {
            preStart();
            waitForStart();
            act();
        }
        catch (Exception ex){
            telemetry.addData("Issues autonomous initialization", ex);
        }

        telemetry.update();

    }

    protected void preStart(){

    }

    protected void initRobot(){
        try{
            robot.init(this,this.hardwareMap, telemetry);
        }
        catch (Exception ex){
            telemetry.addData("Init", ex.getMessage());
        }
    }

    protected void act(){

    }

}
