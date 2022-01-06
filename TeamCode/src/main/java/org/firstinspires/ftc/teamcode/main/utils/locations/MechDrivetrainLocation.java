package org.firstinspires.ftc.teamcode.main.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.utils.interactions.groups.StandardMechanumRobotDrivetrain;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

public class MechDrivetrainLocation extends Location {

    public enum Action {
        SET_SPEED,
        SET_VOLTAGE,
        MOVE_DISTANCE_IN_INCHES
    }

    private StandardMechanumRobotDrivetrain DRIVETRAIN;

    public MechDrivetrainLocation(HardwareMap hardware) {
        try {
            StandardMotor rt = new StandardMotor(hardware, Resources.Drivetrain.Motors.Driving.RightTop, DcMotorSimple.Direction.FORWARD, 1120, 1, 2);
            StandardMotor rb = new StandardMotor(hardware, Resources.Drivetrain.Motors.Driving.RightBottom, DcMotorSimple.Direction.FORWARD, 1120, 1, 2);
            StandardMotor lt = new StandardMotor(hardware, Resources.Drivetrain.Motors.Driving.LeftTop, DcMotorSimple.Direction.FORWARD, 1440, 1, 2);
            StandardMotor lb = new StandardMotor(hardware, Resources.Drivetrain.Motors.Driving.LeftBottom, DcMotorSimple.Direction.FORWARD, 1120, 1, 2);
            DRIVETRAIN = new StandardMechanumRobotDrivetrain(rt, rb, lt, lb);
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
