package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.IMU;

public class GyroSubsystem extends SubsystemBase
{
    public final IMU m_imu;

    public GyroSubsystem(IMU imu)
    {
       this.m_imu = imu;
    }

    public void resetGyro()
    {
        this.m_imu.resetYaw();
    }
}
