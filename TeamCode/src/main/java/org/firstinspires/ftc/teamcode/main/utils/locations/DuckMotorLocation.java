package org.firstinspires.ftc.teamcode.main.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

public class DuckMotorLocation extends Location {

    public enum Action {
        SET_SPEED,
        SET_VOLTAGE,
        MOVE_DISTANCE_IN_INCHES
    }

    private StandardMotor MOTOR;

    public DuckMotorLocation(HardwareMap hardware) {
        try {
            MOTOR = new StandardMotor(hardware, Resources.DuckSpinner.Motors.Spinner, DcMotorSimple.Direction.FORWARD, 1440, 1, 2);
        } catch(Exception ignored) {};
    }

    public void handleInput(Action action, int input) {
        if(MOTOR == null) {
            return;
        }
        switch(action) {
            case SET_SPEED:
                MOTOR.driveWithEncoder(input);
                break;
            case SET_VOLTAGE:
                MOTOR.driveWithoutEncoder(input);
                break;
            case MOVE_DISTANCE_IN_INCHES:
                MOTOR.driveDistance(input, 50);
                break;
        }
    }

    @Override
    public void stop() {
        if(MOTOR == null) {
            return;
        }
        MOTOR.stop();
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
        return MOTOR;
    }

}
