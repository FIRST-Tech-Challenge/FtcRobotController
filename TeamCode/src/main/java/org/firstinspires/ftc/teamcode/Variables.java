package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Variables {
    //declare motors
    public static DcMotor motorFL;          //motor01
    public static DcMotor motorBL;          //motor02
    public static DcMotor motorFR;          //motor03
    public static DcMotor motorBR;          //motor04
    public enum Direction {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,
        ROTATE_LEFT,
        ROTATE_RIGHT,
    }


    //other variables
    public static double clicksPerRotation = 537.6;
    public static double rotationsPerMeter = 1/0.3015928947;
}
