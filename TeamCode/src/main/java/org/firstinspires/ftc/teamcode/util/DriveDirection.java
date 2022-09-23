package org.firstinspires.ftc.teamcode.util;

public class DriveDirection {

    public double strafe;
    public double drive;
    public DriveDirection(double drive,double strafe) {
        this.drive = drive;
        this.strafe = strafe;



    }
    public void rotate(double angle){
        double rads = -angle * Math.PI / 180.0;
        double od = this.drive;
        double os = this.strafe;
        this.drive  = Math.cos(rads) * od  +-Math.sin(rads) * os;
        this.strafe =  Math.sin(rads) * od - Math.cos(rads) * os ;
    }
}

