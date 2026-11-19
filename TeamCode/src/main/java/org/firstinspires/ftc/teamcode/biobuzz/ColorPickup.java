package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Color steering is opt-in and limited to active pickup strokes, never parking. */
public final class ColorPickup {
    private ColorPickup() { }

    public static void steer(Sensors sensors,double now,Outputs out,boolean collectBlue,boolean collectRed) {
        if(out.intakePower<=0||out.forward<0||out.shooterPower!=0||out.feederPower!=0)return;
        Vision vision=sensors.vision;
        if(!sensors.cameraConnected||!vision.fresh(now,1)||now-vision.capturedAt>.25
                ||!sensors.odometryValid||sensors.pose==null||!sensors.pose.finite()
                ||now-sensors.sampledAt<0||now-sensors.sampledAt>.25)return;
        Target target=null;
        for(Target candidate:vision.targets){
            if(candidate.id>=0||!Double.isFinite(candidate.tx)||!Double.isFinite(candidate.areaPercent)
                    ||candidate.areaPercent<=0||!("yellow".equals(candidate.color)||"blue".equals(candidate.color)
                    ||"red".equals(candidate.color)))continue;
            if(target==null||candidate.areaPercent>target.areaPercent)target=candidate;
        }
        if(target==null)return;
        boolean collect="yellow".equals(target.color)||("blue".equals(target.color)&&collectBlue)
                ||("red".equals(target.color)&&collectRed);
        out.strafeRight=0;
        if(collect){
            out.turnClockwise=Math.abs(target.tx)<=2?0:FieldLayout.clip(target.tx*.02,.20);
        }else{
            out.forward=0;out.intakePower=0;
            out.turnClockwise=target.tx>=0?-.15:.15;
        }
    }
}
