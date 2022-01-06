package org.firstinspires.ftc.teamcode.main.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.utils.interactions.groups.StandardCarfaxVehicleDrivetrain;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

public class CarfaxDrivetrainLocation extends Location {

    public enum Action {
        SET_SPEED,
        SET_VOLTAGE,
        MOVE_DISTANCE_IN_INCHES
    }

    private StandardCarfaxVehicleDrivetrain DRIVETRAIN;

    public CarfaxDrivetrainLocation(HardwareMap hardware) {
        try {
            StandardMotor rt = new StandardMotor(hardware, Resources.Drivetrain.Motors.Driving.RightTop, DcMotorSimple.Direction.FORWARD, 1440, 1, 2);
            StandardMotor lt = new StandardMotor(hardware, Resources.Drivetrain.Motors.Driving.RightBottom, DcMotorSimple.Direction.FORWARD, 1440, 1, 2);
            DRIVETRAIN = new StandardCarfaxVehicleDrivetrain(rt, lt);
        } catch(Exception ignored) {}
    }

    public void handleInput(Action action, int rightInput, int leftInput) {
        if(DRIVETRAIN == null) {
            return;
        }
        switch(action) {
            case SET_SPEED:
                DRIVETRAIN.driveWithEncoder(rightInput, leftInput);
                break;
            case SET_VOLTAGE:
                DRIVETRAIN.driveWithoutEncoder(rightInput, leftInput);
                break;
            case MOVE_DISTANCE_IN_INCHES:
                DRIVETRAIN.driveDistance(rightInput, leftInput, 50);
                break;
        }
    }

    @Override
    public void stop() {
        if(DRIVETRAIN == null) {
            return;
        }
        DRIVETRAIN.stop();
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
        return DRIVETRAIN;
    }

}
