package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

/**
 * Basic driveTrain class that have common functions usable for both auto & teleop
 * <p>The drive train should only hold the hardware related reference and method, all the
 * driving logic will be in op mode classes</p>
 */
public abstract class BasicDriveTrain extends SubsystemBase {

    protected static final double OdometryWheelCircumference = 150.792;

    // default to no power
    protected double powerRatio = 0;

    // 1 means drive forward, -1 means drive backward
    protected double directionFlag = 1;

    protected final GamepadEx gamepad;
    protected MecanumDrive drive;
    protected final Telemetry telemetry;

    protected DriverFeedback feedback;

    protected boolean revertMotor = false;


    public BasicDriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback, boolean revertMotor) {

        this.revertMotor = revertMotor;
        /* instantiate motors */
        createAndInitHardwares(hardwareMap);

        createDrive();

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

//        this.fL.encoder.reset();
    }

    protected abstract void createDrive();

    protected abstract void createAndInitHardwares(HardwareMap hardwareMap);

    public void stop() {
        drive.stop();
    }

    public void ToggleDriveDirection() {

        directionFlag = directionFlag * -1d;

        if(isDrivingForward()) {
            feedback.DriverControllerGreen();
        } else {
            feedback.DriverControllerRed();
        }

        feedback.DriverRumbleBlip(1);
    }

    public void setPowerRatio(double powerRatio) {
        this.powerRatio = powerRatio;
    }

    private boolean isDrivingForward() {
        return directionFlag > 0;
    }

//    /// ---
//    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
//        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
//    }

//    private double GetOdometryDistance() {
//        return OdometryWheelCircumference * this.fL.encoder.getPosition() / 2000;
//    }
//
//    private double GetStrafeDistance() {
//        return OdometryWheelCircumference * this.bL.encoder.getPosition() / 2000;
//    }

}