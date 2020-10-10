package org.firstinspires.ftc.teamcode.pathgen;

import java.util.ArrayList;

import static java.lang.Math.abs;

public class Path extends ArrayList<PathPoint> {
    PathPoint start;
    PathPoint end;

    public static Path ofPoints(double startAngle, double endAngle, PathPoint... points){
        Path ret = new Path();

        ret.start = new PathPoint(30, 0).rotate(startAngle + Math.PI).translate(points[0]);
        ret.end = new PathPoint(30, 0).rotate(endAngle).translate(points[points.length-1]);
        //last point angle is preset
        points[points.length - 1].dir = points[points.length - 1].angTo(ret.end);
        points[points.length - 1].isLast = true;

        for(PathPoint p:points){
            ret.insert(p);
            p.isControl = true;
        }

        return ret;
    }

    void subdivide(boolean tightTurnsOnly){
        for(int i = 1; i < size()-1; i++){

            if(!tightTurnsOnly || abs(get(i).ang()) < Math.PI * 0.7){
                //subdivide
                insert(PathPoint.midPoint(get(i+1), get(i)), i+1);
                insert(PathPoint.midPoint(get(i-1), get(i)), i);
                i+=2;
            }
        }
    }

    void velocities(){
        for(PathPoint p:this){
            p.velocities();
            p.speed =  p.avgRotation() / Math.PI;
            p.speed = Math.pow(p.speed, 4);
        }
    }

    // The last point has special properties
    void updateEnds(){
        // remove dir lock from old
        for(PathPoint p:this){
            p.isLast = false;
        }
        //add new dir lock
        get(this.size() - 1).dir = get(this.size() - 1).angTo(end);
        get(this.size() - 1).isLast = true;
    }

    void move(double multiplier){
        for(int i = 1; i < size()-1; i++){
            get(i).move(multiplier);
        }
    }


    public boolean insert(PathPoint p){
        insert(p, size());
        return true;
    }

    // yeah, a linked list would be more efficient but this
    // is less complicated
    public void insert(PathPoint p, int index){

        PathPoint prevP = null;
        try {prevP = get(index - 1);}catch(IndexOutOfBoundsException e){}
        PathPoint nextP = null;
        try {nextP = get(index);}catch(IndexOutOfBoundsException e){}

        if(size() == 0){
            p.prev = start;
            p.next = end;
        }else if(index == 0){
            p.prev = start;
            p.next = nextP;
            nextP.prev = p;
        }else if (index==size()){
            p.prev = prevP;
            p.next = end;
            prevP.next = p;
        }else{
            p.prev = prevP;
            p.next = nextP;
            prevP.next = p;
            nextP.prev = p;
        }

        add(index, p);
    }

}
