package org.firstinspires.ftc.teamcode.robots.catbot;

import com.acmerobotics.dashboard.config.Config;

@Config("IronGiantGameVariables")
class ConeStack {
    int coneNum = 5;
    public static int TICKSTOINCH = 100;//this needs to get a real value 50 is a place holder
    public static int ticksPerCone = (int)(TICKSTOINCH * 1.4);
    public int takeCone(){
        return coneNum--;
    }

    public int height() {return (takeCone() * ticksPerCone);}
}
