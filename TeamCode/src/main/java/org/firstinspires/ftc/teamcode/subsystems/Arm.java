package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    //Arm states, may be converted to a boolean should there be no other necessary states
    private enum ARM_STATES{
        UP,
        DOWN
    }

    //Adjustable Constants
    public double ARM_SPEED = 0.01; //Servos range from [0,1]. Start low and progress faster as needed
    public double DOWN_POSITION = 0;
    public double UP_POSITION = 1;

    //Internal variables
    private Servo armLeft, armRight;
    private ARM_STATES armState;


    /**
     * A quick setup constructor for the Arm Class
     * @param hw [HardwareMap] The hardware map used to initialize the servos
     */
    public Arm(HardwareMap hw){
        this(hw, "armLeft", "armRight");
    }

    /**
     * Primary constructor to create an Arm Class subsystem
     * @param hw [HardwareMap] The hardware map used to initialize the servos
     * @param nameLeft [String] Name of the left servo used in the configuration
     * @param nameRight [String] Name of the right servo used in the configuration
     */
    public Arm(HardwareMap hw, String nameLeft, String nameRight){
        armLeft = hw.get(Servo.class, nameLeft);
        armRight = hw.get(Servo.class, nameRight);
        armRight.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * A function primarily for testing purposes to find optimal positions for the servo arm.
     * @param power [double] Can be paired with a trigger or joystick to dynamically adjust the speed of the arm going up/down
     */
    public double changeHeight(double power){
        double newPos = armLeft.getPosition() + ARM_SPEED * power;
        armLeft.setPosition(newPos);
        armRight.setPosition(newPos);
        return newPos;
    }

    /**
     * @return [double] Returns the position of the left arm servo.
     */
    public double getPosition(){
        return armLeft.getPosition();
    }

    /**
     * Sets the servos to a predefined [DOWN_POSITION]
     */
    public void goDown(){
        armLeft.setPosition(DOWN_POSITION);
        armRight.setPosition(DOWN_POSITION);
        armState = ARM_STATES.DOWN;
    }


    /**
     * Sets the servos to a predefined [UP_POSITION]
     */
    public void goUp(){
        armLeft.setPosition(UP_POSITION);
        armRight.setPosition(UP_POSITION);
        armState = ARM_STATES.UP;
    }

    /**
     * Depending on the last state the arm was in,
     * this will set the arm to the opposite position
     */
    public void toggleUpDown(){
        switch(armState){
            case UP:
                goDown();
                break;
            case DOWN:
                goUp();
                break;
            default:
        }

    }



}
