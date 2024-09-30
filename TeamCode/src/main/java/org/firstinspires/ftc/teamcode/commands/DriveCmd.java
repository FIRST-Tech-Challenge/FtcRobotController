package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This command is dedicated to a command that controls driving for the Tele-op mode
 */

public class DriveCmd extends CommandBase {

    private final DrivetrainSub drivetrainSub;
    private final GamepadEx gamepad;
    private final DoubleSupplier angleDegrees;
    private final BooleanSupplier fieldCentricity;

    /**
     * This command deals with the driving in teleop.
     *
     * @param drivetrainSubParam The drivetrain sub to be imported
     * @param gamepad1Param The main gamepad to be imported
     * @param angleParam The angle returned by the IMU, implemented by a double supplier because this is run in init, and we need this value to be updated constantly
     * @param fieldCentricityParam Whether we're field centric or not. This is a supplier for the same reason that angleParam is.
     */

    public DriveCmd(DrivetrainSub drivetrainSubParam, GamepadEx gamepad1Param, DoubleSupplier angleParam, BooleanSupplier fieldCentricityParam){
        this.drivetrainSub = drivetrainSubParam;
        gamepad = gamepad1Param;
        angleDegrees = angleParam;
        fieldCentricity = fieldCentricityParam;
        addRequirements(this.drivetrainSub);
    }

    @Override
    public void execute(){
        double brakeMultiplier = 1;
        double rightTrigger=gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        // if precision mode is on (the right trigger is pulled down to some degree)
        if (rightTrigger > 0.05 && rightTrigger < 0.75) {
            brakeMultiplier = 1 - rightTrigger;
            //telemetry.addData("Precise Mode", "On");
            // also if precision mode is on, but it caps brake at 25% after 75% trigger
        } else if (rightTrigger >= 0.75) {
            brakeMultiplier = 0.25;
            //telemetry.addData("Precise Mode", "On");
        }

        if (fieldCentricity.getAsBoolean()) {
            // optional fifth parameter for squared inputs
            drivetrainSub.getDrive().driveFieldCentric(
                    gamepad.getLeftX()*-brakeMultiplier,
                    gamepad.getLeftY()*-brakeMultiplier,
                    gamepad.getRightX()*-brakeMultiplier,
                    angleDegrees.getAsDouble(),   // gyro value passed in here must be in degrees
                    false
            );
        } else {
            // optional fourth parameter for squared inputs
            drivetrainSub.getDrive().driveRobotCentric(
                    gamepad.getLeftX()*-brakeMultiplier,
                    gamepad.getLeftY()*-brakeMultiplier,
                    gamepad.getRightX()*-brakeMultiplier,
                    false
            );
        }
    }
}
