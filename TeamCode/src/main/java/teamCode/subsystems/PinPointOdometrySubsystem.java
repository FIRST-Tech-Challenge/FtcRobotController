package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import teamCode.GoBildaPinpointDriver;

public class PinPointOdometrySubsystem extends SubsystemBase
{
    GoBildaPinpointDriver odo;
    private double oldTime = 0;

    private final GoBildaPinpointDriver m_odo;

    public PinPointOdometrySubsystem(GoBildaPinpointDriver odo)
    {
        this.m_odo = odo;
        odo.setOffsets(68,-178);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }
    public void resetOdo()
    {
        this.m_odo.resetPosAndIMU();
    }
}