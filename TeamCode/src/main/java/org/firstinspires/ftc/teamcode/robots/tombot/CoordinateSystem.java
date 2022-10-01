package org.firstinspires.ftc.teamcode.robots.tombot;

import org.firstinspires.ftc.teamcode.util.Vector2Int;

public class CoordinateSystem {

    private Vector2Int pos; //current position

    public Vector2Int target; //current target

    public CoordinateSystem(int x, int y){
        pos = new Vector2Int(x,y);
    }

    public void updatePos(int x, int y){ //updates current position
        pos = new Vector2Int(x,y);
    }

    public void setTarget(int x, int y){ //sets new target
        target.x = x;
        target.y = y;
    }


    public int deltaX(){
        return target.x - pos.x;
    }

    public int deltaY(){
        return target.y - pos.y;
    }

    public int x(){
        return pos.x;
    }

    public int y(){
        return pos.y;
    }

    public Vector2Int getPosition(){
        return pos;
    }

    public Vector2Int getTarget(){
        return target;
    }
}


