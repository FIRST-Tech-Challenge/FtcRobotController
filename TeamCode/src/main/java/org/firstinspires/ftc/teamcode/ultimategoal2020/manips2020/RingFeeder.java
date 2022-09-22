package org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

public class RingFeeder implements EbotsManip2020 {

    private Servo ringFeederMotor;
    private StopWatch ringFeederCycleTimer = new StopWatch();


    final long CYCLE_TIME = 500;    // intended to be time to move between positions
    final double RECEIVE = 0.06;
    final double FEED =    0.37;


    public RingFeeder(HardwareMap hardwareMap){
        ringFeederMotor = hardwareMap.get(Servo.class, "ringFeeder");
    }

    @Override
    public void handleGamepadInput(Gamepad gamepad) {
        // ************     RING FEEDER     **********************
        // ring feeder servo should cycle between 2 positions: RECEIVE and FEED
        // time is used to control cycle
        // cycle is triggered using right trigger

        double inputThreshold = 0.3;

        boolean triggerPressed = Math.abs(gamepad.right_trigger) > inputThreshold;
        //  readyToReceiveRing makes sure that the servo is back to the original position before cycling again
        //  also, conveyors shouldn't feed into shooter if not ready for feed
        boolean cycleTimeout = (ringFeederCycleTimer.getElapsedTimeMillis() > (2*CYCLE_TIME));
        double errorFromReceivePosition = ringFeederMotor.getPosition() - RECEIVE;
        double errorFromFeedPosition = ringFeederMotor.getPosition() - FEED;

        if(triggerPressed  && (cycleTimeout)){
            ringFeederCycleTimer.reset();
            ringFeederMotor.setPosition(FEED);
        } else if(ringFeederCycleTimer.getElapsedTimeMillis() > CYCLE_TIME){
            ringFeederMotor.setPosition(RECEIVE);
        }

    }

    public double getPosition(){
        return this.ringFeederMotor.getPosition();
    }

    @Deprecated
    public void setPosition(double targetPosition){
        this.ringFeederMotor.setPosition(targetPosition);
    }

    @Override
    public void stop() {
        //  Find the current position and set as target position
        ringFeederMotor.setPosition(ringFeederMotor.getPosition());
    }

    public void feedRing(){
        ringFeederCycleTimer.reset();
        ringFeederMotor.setPosition(FEED);
        while(ringFeederCycleTimer.getElapsedTimeMillis() < CYCLE_TIME){
            //wait for a cycle
        }
        ringFeederMotor.setPosition(RECEIVE);
    }

}
