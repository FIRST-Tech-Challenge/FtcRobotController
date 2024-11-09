package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.hardware.Servo;

public class PinchBot extends PivotBot{
    static final double VERTICAL_OFFSET = 0;
    static final double VERTICAL_PROPORTION = 1;
    static final double HORIZONTAL_PROPORTION = 1;
    static final double ROTATIONAL_PROPORTION = 1;
    static final double CENTER_POSITION = 0;

    public boolean isOpen = false;

    public Servo pinch;
    public Servo rotate;

    public PinchBot(LinearOpMode opMode) {
        super(opMode);
    }


    public void init(HardwareMap hardwareMap){
        super.init(hardwareMap);
        pinch = hardwareMap.get(Servo.class, "pinch");
        rotate = hardwareMap.get(Servo.class, "rotate");
    }

    public void pinchControl(){
        isOpen = !isOpen;
        if(isOpen){
            pinch.setPosition(0.5);
        }
        if(!isOpen){
            pinch.setPosition(0);
        }
    }
    public void rotate(double angle){
        angle = angle;
        rotate.setPosition(angle);
    }
    public void pickUp(boolean button){
        // use horizontalDistance() to move robot and verticalDistance() to move slide
        // figure out how to find angle and use rotate(angle)
        //double[] position = detectOne();
        //double x = position[0];
        //double y = position[1];
        //double theta = position[2];
        //slideMove((int) ((y + VERTICAL_OFFSET)*VERTICAL_PROPORTION)); //move slide vertically
        //driveStraightByDistance(90, x*HORIZONTAL_PROPORTION, 2);
        //rotate(theta * ROTATIONAL_PROPORTION);
        isOpen = false;
        //pinchControl();

        //slideControl(false, true);
        //rotate(CENTER_POSITION);

    }
}
