package org.firstinspires.ftc.teamcode.main.utils.interactions.groups;

import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardMotor;

/**
 * A StandardVehicleDrivetrain represents the motors which control movement of the robot in a way similar to a real-life vehicle, controlling each side of the motors on its own.
 */
public abstract class StandardVehicleDrivetrain extends StandardDrivetrain {

    public abstract void driveDistance(int rightDistance, int leftDistance, int speed);

    public abstract void driveDistance(int distance, int speed);

    public abstract void driveWithEncoder(int rightSpeed, int leftSpeed);

    public abstract void driveWithEncoder(int speed);

    public abstract void driveWithoutEncoder(int rightPower, int leftPower);

    public abstract void driveWithoutEncoder(int power);

    public abstract StandardMotor getRightTop();

    public abstract StandardMotor getRightBottom();

    public abstract StandardMotor getLeftTop();

    public abstract StandardMotor getLeftBottom();
}
