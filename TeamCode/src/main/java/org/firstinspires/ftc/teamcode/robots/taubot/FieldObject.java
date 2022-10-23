package org.firstinspires.ftc.teamcode.robots.taubot;

import org.firstinspires.ftc.teamcode.util.Vector2;

public class FieldObject {
    private Vector2 pos; //current position

    private String name = "";
    private int height = 0;
    public boolean neutral;
    public boolean redOwn;
    public boolean blueOwn;

    public FieldObject(String n, double x, double y, int h){
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
        return pos.y;
    }

    public int getHeight(){ return height; }

    public String getName(){ return name; }

    public Vector2 getPosition(){
        return pos;
    }
}
