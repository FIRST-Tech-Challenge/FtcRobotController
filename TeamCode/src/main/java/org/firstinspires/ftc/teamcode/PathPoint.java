package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// Creates a destination in X Y Heading(or H) and time to get to location in seconds and returns values when asked
public class PathPoint {

    private int x;
    private int y;
    private int h;
    private int timeLim;

    public PathPoint(int x, int y, int h, int timeLim){

        this.x = x; // distance x in cm from robot starting point
        this.y = y; // distance y in cm from robot starting point
        this.h = h; // degree in radian that robot want to rotate to (counter clock-wise)
        this.timeLim = timeLim; // limit for robot to attempt to get to point (in milliseconds) until time is up
    }

    public int giveX(){
        return this.x;
    }

    public int giveY(){
        return this.y;
    }

    public int giveH(){
        return this.h;
    }

    public int giveTimeLim(){
        return this.timeLim;
    }

}