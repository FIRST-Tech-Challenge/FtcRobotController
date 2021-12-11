package org.firstinspires.ftc.teamcode.competition.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.groups.StandardMechanum;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.groups.StandardTank;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardMotor;

public class MechDrivetrainLocation extends Location {

    public enum Action {
        SET_SPEED,
        SET_VOLTAGE,
        MOVE_DISTANCE_IN_INCHES
    }

    private StandardMechanum DRIVETRAIN;

    public MechDrivetrainLocation(HardwareMap hardware) {
        try {
            StandardMotor rt = new StandardMotor(hardware, hardware.appContext.getString(R.string.DRIVETRAIN_RIGHT_TOP_DRIVING_MOTOR), DcMotorSimple.Direction.FORWARD, 1120, 1, 2);
            StandardMotor rb = new StandardMotor(hardware, hardware.appContext.getString(R.string.DRIVETRAIN_RIGHT_BOTTOM_DRIVING_MOTOR), DcMotorSimple.Direction.FORWARD, 1120, 1, 2);
            StandardMotor lt = new StandardMotor(hardware, hardware.appContext.getString(R.string.DRIVETRAIN_LEFT_TOP_DRIVING_MOTOR), DcMotorSimple.Direction.FORWARD, 1440, 1, 2);
            StandardMotor lb = new StandardMotor(hardware, hardware.appContext.getString(R.string.DRIVETRAIN_LEFT_BOTTOM_DRIVING_MOTOR), DcMotorSimple.Direction.FORWARD, 1120, 1, 2);
            DRIVETRAIN = new StandardMechanum(rt, rb, lt, lb);
        } catch(Exception ignored) {}
    }

    public void handleInput(Action action, int rightTopInput, int rightBottomInput, int leftTopInput, int leftBottomInput) {
        if(DRIVETRAIN == null) {
            return;
        }
        switch(action) {
            case SET_SPEED:
                DRIVETRAIN.driveWithEncoder(rightTopInput, rightBottomInput, leftTopInput, leftBottomInput);
                break;
            case SET_VOLTAGE:
                DRIVETRAIN.driveWithoutEncoder(rightTopInput, rightBottomInput, leftTopInput, leftBottomInput);
                break;
            case MOVE_DISTANCE_IN_INCHES:
                DRIVETRAIN.driveDistance(rightTopInput, rightBottomInput, leftTopInput, leftBottomInput, 50);
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
