package org.firstinspires.ftc.teamcode.opmodes.autonomous.driver;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoFourWheelMecanumDriveTrain;

/**
 * this is base class of robot driver, it hold a reference to driveTrain and have abstract method
 * {@link #drive(double, double, double)} for child class to implement for drive style of driving
 *
 */
public abstract class AutoRobotDriver {

    protected final AutoFourWheelMecanumDriveTrain driveTrain;

    public AutoRobotDriver(AutoFourWheelMecanumDriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    public void stop() {
        driveTrain.stop();
    }

    public abstract void drive(double strafeSpeed, double forwardSpeed, double turnSpeed);
}
