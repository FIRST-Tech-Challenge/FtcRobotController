package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;
import org.firstinspires.ftc.teamcode.subsystems.ImuSub;

/**
 * This command is dedicated to turning with a certain angle
 */

public class TurnCmd extends CommandBase {

    private final DrivetrainSub drivetrainSub;
    private final double turnAngle;
    private final double turnSpeed;
    private Telemetry telemetry;
    private ImuSub imu;

    /**
     * Turns at a certain angle and speed importing the IMU and telemetry.
     *
     * @param turnAngleParam
     * @param turnSpeedParam
     * @param drivetrainSubParam
     * @param imuParam
     * @param telemetryParam
     */

    public TurnCmd(double turnAngleParam, double turnSpeedParam, DrivetrainSub drivetrainSubParam, ImuSub imuParam, Telemetry telemetryParam) {
        turnAngle = turnAngleParam;
        turnSpeed = turnSpeedParam;
        drivetrainSub = drivetrainSubParam;
        imu = imuParam;
        telemetry = telemetryParam;

        addRequirements(drivetrainSubParam);
    }

    @Override
    public void initialize() {
        drivetrainSub.resetEncoders();

        if (turnAngle>=0) {
            drivetrainSub.move(0, turnSpeed);
        }else{
            drivetrainSub.move(0, -turnSpeed);
        }
        if ((turnAngle == 0) && (turnSpeed == 0 )){
            telemetry.addLine("Both turn angle and turn speed are 0!");
        }
        imu.resetAngle();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSub.getDrive().stop();
    }


    @Override
    public boolean isFinished() {
        //System.out.println("IMU Angle: " + imu.getAngle());
        if (turnAngle>0) {
            //System.out.println("Is done? " + (imu.getAngle() >= turnAngle));
            return imu.getAngle() >= turnAngle;
        }else{
            //System.out.println("Is done? " + (imu.getAngle() <= turnAngle));
            return imu.getAngle() <= turnAngle;
        }

    }

}