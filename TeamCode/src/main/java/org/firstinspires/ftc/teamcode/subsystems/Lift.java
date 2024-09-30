package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    //Adjustable Constants
    public int LIFT_SPEED = 20; //ticks per call
    public double POWER = 1; //Max Power
    public int LOW_HEIGHT = 0; //Ticks
    public int MEDIUM_HEIGHT = 1000; //Ticks
    public int HIGH_HEIGHT = 2000; //Ticks

    //Internal variables
    private DcMotor liftLeft, liftRight;
    private int targetPosition;


    /**
     * Quick Constructor for the Lift Subsystem Class
     * @param hw [HardwareMap] Hardware map necessary to initialize the motors.
     */
    public Lift(HardwareMap hw){
        this(hw, "liftLeft", "liftRight");
    }

    /**
     * Primary constructor for the Lift Subsystem Class
     * @param hw [HardwareMap] Hardware map necessary to initialize motors.
     * @param nameLeft [String] Name of the left motor assigned in the configuration.
     * @param nameRight [String] Name of the right motor assigned in the configuration.
     */
    public Lift(HardwareMap hw, String nameLeft, String nameRight){
        //Initialize motors
        this.liftLeft = hw.get(DcMotor.class, nameLeft);
        this.liftRight = hw.get(DcMotor.class, nameRight);
        //Reverse one of the motors
        this.liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //Ensures motor encoders are reset
        resetLift(liftLeft);
        resetLift(liftRight);
    }

    /**
     * Manual tester to adjust the height of the lift.
     * @param power [double] Can be paired with a joystick or trigger to have a dynamic speed to raise the lift.
     * @return [int] Returns the new target position of the lift in ticks.
     */
    public int moveLift(double power){
        targetPosition += (power * LIFT_SPEED);
        liftLeft.setTargetPosition(targetPosition);
        liftRight.setTargetPosition(targetPosition);
        return targetPosition;
    }

    /**
     * Tester function to track current height.
     * @return [int] Returns current position of the left motor in ticks.
     */
    public int getPosition(){
        return liftLeft.getCurrentPosition();
    }


    /**
     * Sets the motors' target position to [LOW_HEIGHT]
     */
    public void goToLow(){
        liftLeft.setTargetPosition(LOW_HEIGHT);
        liftRight.setTargetPosition(LOW_HEIGHT);
    }
    /**
     * Sets the motors' target position to [MEDIUM_HEIGHT]
     */
    public void goToMedium(){
        liftLeft.setTargetPosition(MEDIUM_HEIGHT);
        liftRight.setTargetPosition(MEDIUM_HEIGHT);
    }

    /**
     * Sets the motors' target position to [HIGH_HEIGHT]
     */
    public void goToHigh(){
        liftLeft.setTargetPosition(HIGH_HEIGHT);
        liftRight.setTargetPosition(HIGH_HEIGHT);
    }




    /**
     * Resets the motor encoder of the passed motor.
     * @param liftMotor [DcMotor] Motor encoder that should be reset.
     */
    private void resetLift(DcMotor liftMotor){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = liftMotor.getCurrentPosition();
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(POWER);
    }
}
