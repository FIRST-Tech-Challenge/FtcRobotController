package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakePivotSubsystem extends SubsystemBase
{
    private final Servo m_intakePivotServo;

    public IntakePivotSubsystem(HardwareMap hMap, String name )
    {
        this.m_intakePivotServo = hMap.get(Servo.class, name);
    }

    // Pivots position of the intake.
    public void pivotIntake(double pos)
    {
        this.m_intakePivotServo.setPosition(pos);
    }
}
