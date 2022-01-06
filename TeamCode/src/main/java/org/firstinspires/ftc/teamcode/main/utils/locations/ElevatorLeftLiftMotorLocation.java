package org.firstinspires.ftc.teamcode.main.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

public class ElevatorLeftLiftMotorLocation extends Location {

    public enum Action {
        SET_SPEED,
        SET_VOLTAGE,
        MOVE_DISTANCE_IN_INCHES
    }

    private StandardMotor LEFT;

    public ElevatorLeftLiftMotorLocation(HardwareMap hardware) {
        try {
            LEFT = new StandardMotor(hardware, Resources.Elevator.Motors.LeftLift, DcMotorSimple.Direction.REVERSE, 288, 1, 1);
        } catch(Exception ignored) {}
    }

    public void handleInput(Action action, int input) {
        if(LEFT == null) {
            return;
        }
        switch(action) {
            case SET_SPEED:
                LEFT.driveWithEncoder(input);
                break;
            case SET_VOLTAGE:
                LEFT.driveWithoutEncoder(input);
                break;
            case MOVE_DISTANCE_IN_INCHES:
                LEFT.driveDistance(input, 50);
                break;
        }
    }

    @Override
    public void stop() {
        if(LEFT == null) {
            return;
        }
        LEFT.stop();
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
        return LEFT;
    }

}
