package org.firstinspires.ftc.teamcode.robots.taubot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.Vector2;

public class FieldObject {
    private Vector2 pos; //current position

    private String name = "";
    private int height = 0;
    public boolean neutral;
    public boolean redOwn;
    public boolean blueOwn;

    public FieldObject(String n, double y, double x, int h){
        neutral = true;
        redOwn = false;
        blueOwn = false;
        name = n;
        pos = new Vector2(x,y);
        height = h;
    }

    public void setOwnership(boolean isBlue, boolean isRed){
        if(!isBlue && !isRed)neutral = true;
        if(isBlue)isBlue = true;
        if(isRed)isRed = true;
    }

    public double x(){
        return pos.x;
    }

    public double y(){
        return -pos.y;
    }

    public double z() {
        switch (height) {
            case 0:
                return 7;
            case 1:
                return 5;
            case 2:
                return 17;
            case 3:
                return 27;
            case 4:
                return 36;
            default:
                return 36;
        }
    }

    public int getHeight(){ return height; }

    public String getName(){ return name; }

    public Pose2d getPosition(){
        return new Pose2d(pos.x,pos.y);
    }

    public Vector2 getPositionVec(){
        return pos;
    }
}
