package org.firstinspires.ftc.teamcode.robots.catbot;

import com.acmerobotics.dashboard.config.Config;

@Config("IronGiantGameVariables")
class ConeStack {
    int coneNum = 5;
    public static int TICKSTOINCH = 50;//this needs to get a real value 50 is a place holder
    public int takeCone(){
        return coneNum--;
    }

    public int height(){
        return (int)(TICKSTOINCH*(14 - 1.4*(5 - takeCone())));
    }
}
