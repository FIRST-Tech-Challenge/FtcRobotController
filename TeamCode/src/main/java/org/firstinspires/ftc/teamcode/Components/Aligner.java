package org.firstinspires.ftc.teamcode.Components;

public class Aligner {
    public Aligner() {

    }
    //extend the aligner
    public void extendAligner() {
        //no input TODO:There should be an input of distance, as well as speed, unless you want seperate function for setPosition
        //state has to be not already extending aligner TODO:if claw is closed and not yet raised, no extend, this is an asynchronous function
        //TODO: , it can extend when extending, just not when extended
        //set motor velocity to preset speed TODO: Refer to line 9
        //set state extending aligner
        //log to general robot log that it is now extending the aligner through function extendAligner() TODO: Only if not already extending
    }

    //retract the aligner
    public void retractAligner() {
        //no input
        //state has to be not already retracting aligner TODO: same as previous func, this is an async func
        //set motor velocity to preset speed TODO:this MUST be an optimized setPosition function
        //set state retracting aligner
        //log to general robot log that it is now retracting the aligner through function retractAligner() TODO: only log once(ik u alrdy kn)
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
