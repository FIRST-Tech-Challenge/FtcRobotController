package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ClimbArmSubsystem extends SubsystemBase
{
    private final DcMotor m_climberArmMotor;

    public ClimbArmSubsystem(DcMotor climberArmMotor)
    {
        this.m_climberArmMotor = climberArmMotor;
        this.m_climberArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.m_liftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.m_climberArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void climb(int climb)
    {
        this.m_climberArmMotor.setTargetPosition(this.m_climberArmMotor.getCurrentPosition() + climb);
        this.m_climberArmMotor.setPower(1.0);
        this.m_climberArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public boolean atTarget(int target)
    {
        return this.m_climberArmMotor.getCurrentPosition() >= target-5 || this.m_climberArmMotor.getCurrentPosition() >= target+5;
    }

    public void stop()
    {
        this.m_climberArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}