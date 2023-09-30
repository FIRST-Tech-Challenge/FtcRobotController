package org.firstinspires.ftc.teamcode.robots.csbot;

import org.firstinspires.ftc.teamcode.util.Vector2;

public class CoordinateSystem {

    private Vector2 pos; //current position

    public Vector2 target; //current target

    public CoordinateSystem(int x, int y){
        pos = new Vector2(x,y);
    }

    public void updatePos(int x, int y){ //updates current position
        pos = new Vector2(x,y);
    }

    public void setTarget(int x, int y){ //sets new target
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

    public Vector2 calcNormal(Vector2 vec){ //assumes robot always faces inwards except for corners then calulates direction robot is facing when it starts. May not use tho idk
        if(vec.x == vec.y){ // if its in the corners then always faces towards the cone stack
            return new Vector2(-vec.x/Math.abs(vec.x),0);
        }else if(Math.max(vec.x, vec.y) == vec.x){
            return new Vector2(-vec.x/Math.abs(vec.x),0);
        }else{
            return new Vector2(0,-vec.y/Math.abs(vec.y));
        }
    }
}


