package org.firstinspires.ftc.teamcode.ebotsenums;

public enum RobotOrientation {
    FORWARD(0) //Assumes increasing counts facing robot front
    , LATERAL(Math.PI/2)  //Assumes increasing clicks facing robot left (top view)
    , NONE(0);

    double encoderAngleRad;

    RobotOrientation(double inputAngleRad){
        this.encoderAngleRad = inputAngleRad;
    }

    public double getEncoderAngleRad() {
        return encoderAngleRad;
    }
}
