package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odometry.OdoBase;
import org.firstinspires.ftc.teamcode.skills.RingDetector;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;

public abstract class AutoBase extends OdoBase {
    protected ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        try {
            super.runOpMode();
            preStart();
            bot.initDetector();
            telemetry.update();
            waitForStart();
            startLocator();
            act();
            sleep(5000);
        }
        catch (Exception ex){
            telemetry.addData("Error", ex);
            telemetry.update();
            sleep(5000);
        }
    }

    protected void preStart(){

    }

    protected String getModeName(){
        return "";
    }


    protected void act(){

    }

}
