package org.firstinspires.ftc.teamcode.Final.Final.Auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.hardware;

public class calculations {
    private hardware hardware;

    //TIME
    public ElapsedTime timer = new ElapsedTime();

    public double timeToRotate360 = 2.5;
    double makeSpaceForArm = 5;
    double timeToLiftArm = 3;

    //DRIVING
    double driveSpeed = 0.8;
    double turnSpeed = 0.55;

    //MANTIS
    double mantisUp = 1;
    double mantisDown = -0.1;
    double mantisHold = 0.3;

    //HOPPER
    double hopperUp = 1;
    double hopperDown = -0.65;
    double hopperHold = 0.1;

    //LIFT
    double liftUp = 1; //lift up speed
    double liftDown = -1; //lift down speed
    double liftHold = 0; //Holds the lift position

    //GRABBER
    double bottomCollect = -1;
    double topCollect = 1;

    double bottomRelease = 1;
    double topRelease = -1;

    double grabberHold = 0;

    //DOOR
    double open = 0;     // Open door position
    double close = 0.6;// Close door position

    //COLOR
//    double red = hardware.colorSensor.red();
//    double blue = hardware.colorSensor.blue();
//    double green = hardware.colorSensor.green();
//    boolean yellow = (red > (blue + 100)) && (green > (blue + 100));

    //DISTANCE
    double cageDistance = 13;
    double clearCage = 8;
    double smidge = 5;
}
