package org.firstinspires.ftc.teamcode.Components.Aligner;

public class IntakeMotor {
    public IntakeMotor() {

    }

    //spin the motor to align a cone
    public void spinAlignerIntake() {
        //no input
        //state has to be not already spinning it to intake a cone TODO: this is an async func
        //set motor velocity to preset speed
        //set state spinning aligner to intake to true
        //log to general robot log that it is now spinning aligner to intake through function spinAlignerIntake()
    }

    //spin aligner to inputted speed TODO: none of this func
    public void spinAligner() {
        //input of desired velocity to spin the motor
        //no conditions
        //set motor velocity
        //set spinning aligner states to true and false or false and true depending on positive or negative velocity
        //log to general robot log that it is now spinning aligner to intake/reverse intake depending on positive or negative
        //through function spinAligner()
    }

    //spin the motor to reverse out a cone
    public void spinAlignerReverseIntake() {
        //no input
        //state has to be not already spinning it to reverse intake a cone TODO: this is an async func
        //set motor velocity to preset speed
        //set state spinning aligner to reverse intake to true
        //log to general robot log that it is now spinning aligner to reverse the intake through function spinAlignerReverseIntake()
    }

    //stop the motor
    public void stopAligner() {
        //no input
        //state has to be not stopped already TODO: this is an async func
        //set motor velocity to 0
        //set state aligner stopped to true
        //log to general robot log that aligner is now stopped through function stopAligner()
    }
}
