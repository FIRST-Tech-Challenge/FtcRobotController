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
    private final DoubleSupplier driveX;
    private final DoubleSupplier driveY;
    private final DoubleSupplier turnX;
    private final DoubleSupplier turnY;
    private final DoubleSupplier rightTrigger;
    private final DoubleSupplier leftTrigger;
    private final DoubleSupplier angleDegrees;
    private final BooleanSupplier fieldCentricity;

    /**
     * This command deals with the driving in teleop.
     *
     * @param drivetrainSubParam The drivetrain sub to be imported
     * @param angleParam The angle returned by the IMU, implemented by a double supplier because this is run in init, and we need this value to be updated constantly
     * @param fieldCentricityParam Whether we're field centric or not. This is a supplier for the same reason that angleParam is.
     */

    public DriveCmd(DrivetrainSub drivetrainSubParam, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger, DoubleSupplier driveX, DoubleSupplier driveY, DoubleSupplier turnX, DoubleSupplier turnY, DoubleSupplier angleParam, BooleanSupplier fieldCentricityParam){
        this.drivetrainSub = drivetrainSubParam;
        this.driveX = driveX;
        this.driveY = driveY;
        this.turnX = turnX;
        this.turnY = turnY;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        angleDegrees = angleParam;
        fieldCentricity = fieldCentricityParam;
        addRequirements(this.drivetrainSub);
    }

    @Override
    public void execute(){
        double driveX = this.driveX.getAsDouble();
        double driveY = this.driveY.getAsDouble();
        double turnX = this.turnX.getAsDouble();
        double turnY = this.turnY.getAsDouble();
        double brakeMultiplier = 1;
        double rightTrigger = this.rightTrigger.getAsDouble();
        double leftTrigger = this.leftTrigger.getAsDouble();


        if (leftTrigger > 0.05 && leftTrigger < 0.75) {
            brakeMultiplier = (1 - leftTrigger)/2;
        } else if (rightTrigger >= 0.75) {
            brakeMultiplier = 0.125;
        }

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
                    driveX*-brakeMultiplier,
                    driveY*-brakeMultiplier,
                    turnX*-brakeMultiplier,
                    angleDegrees.getAsDouble(),   // gyro value passed in here must be in degrees
                    false
            );
        } else {
            // optional fourth parameter for squared inputs
            drivetrainSub.getDrive().driveRobotCentric(
                    driveX*-brakeMultiplier,
                    driveY*-brakeMultiplier,
                    turnX*-brakeMultiplier,
                    false
            );
        }
    }
}
