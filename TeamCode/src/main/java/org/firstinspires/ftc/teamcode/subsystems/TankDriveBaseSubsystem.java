package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.subsystems.bases.DriveTrainBaseSubsystem;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleSubsystemBase;

// THIS CODE IS OUTDATED AND DIDN'T GET TESTED ON THE NEW ROBOT!
public class TankDriveBaseSubsystem extends SympleSubsystemBase implements DriveTrainBaseSubsystem {
    private boolean invert = false;

    private final MotorEx leftMotor, rightMotor;

    public TankDriveBaseSubsystem(RobotController robotController) {
        super(robotController);
        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Getting motors");
        this.rightMotor = new MotorEx(robotController.getHardwareMap(), "right_wheels");
        this.leftMotor = new MotorEx(robotController.getHardwareMap(), "left_wheels");
        this.rightMotor.setInverted(true);
    }

    public void moveMotors(double leftPower, double rightPower) {
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }

    @Override
    public void moveSideMotors(double left, double right) {
        this.moveMotors(left, right);
    }

    @Override
    public double getForwardDistanceDriven() {
        return this.robotController.getRobotPositionManager().getRightWheelDistanceDriven();
    }

    @Override
    public double getHeading() {
        return this.robotController.getRobotPositionManager().getHeadingByGyro();
    }

    public void setInverted(boolean invert) {
        this.invert = invert;
    }

    public boolean isInverted() {
        return this.invert;
    }
}
