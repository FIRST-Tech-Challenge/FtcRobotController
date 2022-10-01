package org.firstinspires.ftc.teamcode.robots.tombot;

import org.firstinspires.ftc.teamcode.util.Vector2;

public class CoordinateSystem {

    private Vector2 pos; //current position

    public Vector2 target; //current target

    public CoordinateSystem(double x, double y){
        pos = new Vector2(x,y);
    }

    public void updatePos(double x, double y){ //updates current position
        pos = new Vector2(x,y);
    }

    public void setTarget(double x, double y){ //sets new target
        target.x = x;
        target.y = y;
    }


    public double deltaX(){
        return target.x - pos.x;
    }

    public double deltaY(){
        return target.y - pos.y;
    }

    public double x(){
        return pos.x;
    }

    public double y(){
        return pos.y;
    }

    public Vector2 getPosition(){
        return pos;
    }

    public Vector2 getTarget(){
        return target;
    }
}


