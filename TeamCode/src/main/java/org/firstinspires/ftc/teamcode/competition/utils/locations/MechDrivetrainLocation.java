package org.firstinspires.ftc.teamcode.competition.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.groups.StandardTank;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardMotor;

public class MechDrivetrainLocation extends Location {

    public enum Action {
        SET_SPEED,
        SET_VOLTAGE,
        MOVE_DISTANCE_IN_INCHES
    }

    private final StandardTank DRIVETRAIN;

    public MechDrivetrainLocation(HardwareMap hardware) {
        StandardMotor rt = new StandardMotor(hardware, hardware.appContext.getString(R.string.DRIVETRAIN_RIGHT_TOP_DRIVING_MOTOR), DcMotorSimple.Direction.FORWARD);
        StandardMotor rb = new StandardMotor(hardware, hardware.appContext.getString(R.string.DRIVETRAIN_RIGHT_BOTTOM_DRIVING_MOTOR), DcMotorSimple.Direction.FORWARD);
        StandardMotor lt = new StandardMotor(hardware, hardware.appContext.getString(R.string.DRIVETRAIN_LEFT_TOP_DRIVING_MOTOR), DcMotorSimple.Direction.FORWARD);
        StandardMotor lb = new StandardMotor(hardware, hardware.appContext.getString(R.string.DRIVETRAIN_LEFT_BOTTOM_DRIVING_MOTOR), DcMotorSimple.Direction.FORWARD);
        DRIVETRAIN = new StandardTank(rt, rb, lt, lb);
    }

    public void handleInput(Action action, int rightInput, int leftInput) {
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
