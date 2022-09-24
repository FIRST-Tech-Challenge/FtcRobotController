package org.firstinspires.ftc.teamcode.Components;

public class Claw {
    //constructor
    public Claw(){
        //init RFServo & Distance sensor
    }
    //close the claw
    public void closeClaw() {
        //no input
        //the state of claw opened has to be true TODO: Boy better see sumthin with distance
        //set servo position
        //set state of claw closed to true
        //log to general robot log that the claw has been closed through function closeClaw()
    }

    //open the claw
    public void openClaw() {
        //no input
        //the state of claw closed has to be true TODO: refer to line 16
        //set servo position
        //set state of claw open to true
        //log to general robot log that the claw has been opened through function openClaw()
    }
    //look at and return distance to the nearest cone
    public double getConeDistance() {
        //no input
        //no state conditions
        //execute algorithm for observing
        //no setting state
        //log to general robot log that the cone has been observed through function closeClaw()



        //just placeholder so that there are no errors
        return 2.0;
    }
}
