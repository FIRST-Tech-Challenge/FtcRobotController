package org.firstinspires.ftc.teamcode.utils.Math;

import com.qualcomm.robotcore.util.ElapsedTime;


public class ProfileVelAcc {
    public class Stats {
        public double pos,vel,acc,time;

        public Stats(double pos, double vel, double acc, double time) {
            this.pos = pos;
            this.vel = vel;
            this.acc = acc;
            this.time=time;
        }
    }
    
    Stats previousStats;
    public Stats stats;
    ElapsedTime t;
    public ProfileVelAcc(double initialPos,double initialTime){
        t=new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        stats= new Stats(initialPos,initialTime,0,t.milliseconds());
        previousStats= new Stats(initialPos,initialTime,0,t.milliseconds());
    }
    public void calculate(double pos,double time) {
        if(time==previousStats.time){
            return;
        }

        double vel=1000*(pos-previousStats.pos)/(time-previousStats.time);
        double acc=1000*(vel-previousStats.vel)/(time-previousStats.time);

        previousStats.time=stats.time;
        previousStats.pos=stats.pos;
        previousStats.vel=stats.vel;
        previousStats.acc=stats.acc;

        stats.vel=vel;
        stats.acc=acc;
        stats.pos=pos;
        stats.time=time;
    }


    
}
