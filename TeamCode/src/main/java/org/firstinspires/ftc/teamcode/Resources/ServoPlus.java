package org.firstinspires.ftc.teamcode.Resources;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// This class is supposed to make servos easier to program by
// converting the normal 0.0 - 1.0 input into degrees of rotation.
public class ServoPlus {

    // The servo you want to use
    private final Servo servo;

    // the range of motion of the servo
    // this value is the max range assuming that the min value is 0.0
    // so if you gave it 360.0 then the servo has a range of motion of 360.0 degrees,
    // this is from the specs of the servo.
    private final double RANGE_OF_MOTION;

    // the previous angle assigned to the servo, this data is not coming from the servo.
    private double currentAngle;

    private final double rangeMin, rangeMax;

    /**
     * Constructor for this class
     * @param servoInput used to assign the servo this class is targeting
     * @param rangeOfMotionInput used to assign the range of motion variable.
     * @param rangeMinInput used to set the min range of motion
     * @param rangeMaxInput used to set the max range of motion
     */
    public ServoPlus(Servo servoInput, double rangeOfMotionInput, double rangeMinInput, double rangeMaxInput){
        servo = servoInput;
        RANGE_OF_MOTION = rangeOfMotionInput;
        rangeMin = rangeMinInput;
        rangeMax = rangeMaxInput;
    }

    /**
     * The angle you want the servo to turn to
     * @param targetAngle angle in degrees
     */
    public void setServoPos(double targetAngle){

        // ensures you don't send a value that is invalid
        targetAngle = Range.clip(targetAngle, rangeMin, rangeMax);

        // sets the position of the servo to be the ratio of the angle you provided
        // and the full range of motion of the servo.
        servo.setPosition(targetAngle / RANGE_OF_MOTION);

        // updates the currentAngle to reflect your change
        currentAngle = targetAngle;
    }

    /**
     *
     * @return the currentAngle Variable
     */
    public double getServoPos(){
        return currentAngle;
    }

    /**
     * Makes it easier to adjust servos by increments.
     * @param targetAngle angle in degrees
     */
    public void moveServoBy(double targetAngle){
        //feeds the amount you want to move to into the setServoPos.
        setServoPos(targetAngle + currentAngle);
    }

}
