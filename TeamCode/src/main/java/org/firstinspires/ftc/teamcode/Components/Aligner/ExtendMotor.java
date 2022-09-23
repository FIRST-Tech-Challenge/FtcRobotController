package org.firstinspires.ftc.teamcode.Components.Aligner;
//TODO: connect with IntakeMotor class & add updateState funcs
public class ExtendMotor {
    public ExtendMotor() {

    }

    //extend the aligner
    public void extendAligner() {
        //no input TODO:There should be an input of distance, as well as speed, unless you want seperate function for setPosition
        //state has to be not already extending aligner TODO:if claw is closed and not yet raised, no extend, this is an asynchronous function
        //TODO: , it can extend when extending, just not when extended
        //set motor velocity to preset speed TODO: Refer to line 10
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
}
