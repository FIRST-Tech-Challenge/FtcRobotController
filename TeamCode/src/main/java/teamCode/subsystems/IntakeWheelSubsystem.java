package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeWheelSubsystem extends SubsystemBase
{
    private final CRServo m_intakeWheelServo;

    public IntakeWheelSubsystem(CRServo wheel)
    {
        this.m_intakeWheelServo = wheel;
        this.m_intakeWheelServo.setRunMode(Motor.RunMode.VelocityControl);
    }

    // Spins the intake wheel forward, or in reverse.
    public void spinIntake(double speed)
    {
        this.m_intakeWheelServo.set(speed);
    }
    public void spinIntake(int spin)
    {
        this.m_intakeWheelServo.setTargetPosition(spin);
    }
}
