package org.firstinspires.ftc.teamcode.main.utils.locations;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardServo;

public class HandGrabbingServoLocation extends Location {

    public enum Action {
        SET_POSITION,
        STOP,
        START
    }

    private StandardServo SERVO;

    public HandGrabbingServoLocation(HardwareMap hardware) {
        try {
            SERVO = new StandardServo(hardware, hardware.appContext.getString(R.string.HAND_GRABBING_SERVO));
        } catch(Exception ignored) {}
    }

    /**
     * Sets the position of the servo between 0 and 100, or starts or stops it.
     * @param action The action of the servo
     * @param position The position between 0 and 100 if the action was SET_POSITION, otherwise ignored
     */
    public void handleInput(Action action, int position) {
        if(SERVO == null) {
            return;
        }
        switch(action) {
            case SET_POSITION:
                SERVO.setPosition(position);
                break;
            case START:
                SERVO.getController().pwmDisable();
                break;
            case STOP:
                SERVO.getController().pwmEnable();
                break;
        }
    }

    @Override
    public void stop() {
        if(SERVO == null) {
            return;
        }
        SERVO.stop();
    }

    @Override
    public boolean isInputLocation() {
        return true;
    }

    @Override
    public boolean isOutputLocation() {
        return false;
    }

    @Override
    public InteractionSurface getInternalInteractionSurface() {
        return SERVO;
    }

}
