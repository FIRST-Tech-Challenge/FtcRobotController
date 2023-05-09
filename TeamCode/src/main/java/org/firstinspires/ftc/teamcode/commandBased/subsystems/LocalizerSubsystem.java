package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LocalizerSubsystem extends SubsystemBase {

    private RevIMU imu;

    private double heading;

    public LocalizerSubsystem(final HardwareMap hwMap){
        //GYRO
        imu = new RevIMU(hwMap);
        imu.init();
    }

    @Override
    public void periodic() {
        heading = imu.getRotation2d().getDegrees();
    }

    public double getHeading() {
        return heading;
    }
}
