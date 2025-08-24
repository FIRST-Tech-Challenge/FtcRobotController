package org.firstinspires.ftc.teamcode;

public class RobotLocationPractice {
    double angle;

    // constructor method

    public RobotLocationPractice(double angle) {
        this.angle = angle;
    }

    public double getHeading(){
        //this method normalizes robot heading between -180 and 180
        //this is useful for calculating turn angles, especially when crossing the 0,360 boundary

        double angle = this.angle; //copy the angle of the imu
        while (angle > 180){
            angle -= 360  //subtract until in target range
        }
        while (angle <= -180){
            angle += 360  // add until in target range
        }
        return angle; // return normalized value
    }

    public void turnRobot (double angleChange){
        angle += angleChange;
    }

    public void setAngle(double angle){
        this.angle = angle;
    }
}
