package org.firstinspires.ftc.teamcode.autonomous;
import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odometry.OdoBase;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
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
            initLocator();
            waitForStart();
            act();
        }
        catch (Exception ex){
            telemetry.addData("Error", ex);
            telemetry.update();
        } finally {
            stopLocator();
            bot.stopDetection();
        }
    }

    protected void preStart(){

    }


    protected void act(){

    }

    public String getOpModeSide() {
        return opModeSide;
    }

    public void setOpModeSide(String opModeSide) {
        this.opModeSide = opModeSide;
    }

    protected String getModeName() {
        Class<?> klass = this.getClass();
        Autonomous annotation =  klass.getAnnotation(Autonomous.class);
        if (annotation != null && annotation.group().equals("playback")){
            String fullName = annotation.name();
            int numOfDashes = fullName.length() - fullName.replace("-", "").length();
            if (numOfDashes > 1){
                int last = fullName.lastIndexOf("-");
                fullName = fullName.substring(0, last);
            }
            return fullName;
        }
        return "";
    }
}
