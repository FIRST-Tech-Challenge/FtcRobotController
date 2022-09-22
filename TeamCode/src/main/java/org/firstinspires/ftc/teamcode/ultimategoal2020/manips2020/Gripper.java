package org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;

public class Gripper implements EbotsManip2020 {
    private Servo gripper;
    private StopWatch gripperCycleTimer = new StopWatch();
    final double GRIPPER_OPEN = 0.45;
    final double GRIPPER_CLOSED = 0.25;


    public Gripper(HardwareMap hardwareMap){
        gripper = hardwareMap.get(Servo.class, "gripper");

    }

    @Override
    public void handleGamepadInput(Gamepad gamepad) {
        double inputThreshold = 0.3;

        // ************     GRIPPER     **********************
        // left_trigger - toggle between open and closed position

        final long gripperTimeout = 500L;
        boolean gripperToggled = gamepad.left_trigger > inputThreshold
                && gripperCycleTimer.getElapsedTimeMillis() > gripperTimeout;

        if(gripperToggled){
            toggleGripper();
            gripperCycleTimer.reset();
        }
    }

    @Override
    public void stop() {
        // Since this is a servo, set target position to current position
        gripper.setPosition(gripper.getPosition());
    }

    public void toggleGripper(){
        boolean isOpen = Math.abs(gripper.getPosition() - GRIPPER_OPEN) < 0.05;
        if(isOpen){
            gripper.setPosition(GRIPPER_CLOSED);
        } else{
            gripper.setPosition(GRIPPER_OPEN);
        }
    }

    public void closeGripper(){
        gripper.setPosition(GRIPPER_CLOSED);
    }

    @Deprecated
    public double getPosition(){
        return this.gripper.getPosition();
    }

    @Deprecated
    public void setPosition(double targetPosition){
        this.gripper.setPosition(targetPosition);
    }

}
