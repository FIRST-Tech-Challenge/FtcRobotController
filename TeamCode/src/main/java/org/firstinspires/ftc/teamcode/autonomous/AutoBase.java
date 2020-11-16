package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odometry.OdoBase;
import org.firstinspires.ftc.teamcode.skills.RingDetector;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;

public abstract class AutoBase extends OdoBase {
    protected ElapsedTime runtime = new ElapsedTime();
    private String opModeSide = AutoRoute.NAME_RED;

    @Override
    public void runOpMode(){
        try {
            super.runOpMode();
            preStart();
            bot.initDetectorThread(this.getOpModeSide(), this);
            telemetry.update();
            waitForStart();
            startLocator();
            act();
        }
        catch (Exception ex){
            telemetry.addData("Error", ex);
            telemetry.update();
            sleep(5000);
        } finally {
            bot.stopDetection();
        }
    }

    protected void preStart(){

    }

    protected String getModeName(){
        return "";
    }


    protected void act(){

    }

    public String getOpModeSide() {
        return opModeSide;
    }

    public void setOpModeSide(String opModeSide) {
        this.opModeSide = opModeSide;
    }
}
