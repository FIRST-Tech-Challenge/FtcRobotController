package org.firstinspires.ftc.teamcode.competition.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardMotor;

import java.util.ArrayList;

public class ElevatorRightLiftMotorLocation extends Location {

    public enum Action {
        SET_SPEED,
        SET_VOLTAGE,
        MOVE_DISTANCE_IN_INCHES
    }

    private final StandardMotor RIGHT;

    public ElevatorRightLiftMotorLocation(HardwareMap hardware) {
        RIGHT = new StandardMotor(hardware, hardware.appContext.getString(R.string.LIFT_RIGHT_ELEVATOR_MOTOR), DcMotorSimple.Direction.FORWARD);
    }

    public void handleInput(Action action, int input) {
        if(RIGHT == null) {
            return;
        }
        switch(action) {
            case SET_SPEED:
                RIGHT.driveWithEncoder(input);
                break;
            case SET_VOLTAGE:
                RIGHT.driveWithoutEncoder(input);
                break;
            case MOVE_DISTANCE_IN_INCHES:
                RIGHT.driveDistance(input, 50);
                break;
        }
    }

    @Override
    public void stop() {
        if(RIGHT == null) {
            return;
        }
        RIGHT.stop();
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
        return RIGHT;
    }

}
