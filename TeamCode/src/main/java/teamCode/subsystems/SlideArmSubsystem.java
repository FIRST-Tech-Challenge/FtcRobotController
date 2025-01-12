package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SlideArmSubsystem extends SubsystemBase
{
    private final DcMotor m_slideArmMotor;

    public SlideArmSubsystem(DcMotor slideArmMotor)
    {
        this.m_slideArmMotor = slideArmMotor;
        this.m_slideArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.m_slideArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.m_slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.m_slideArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

     public void slideArm(int slide)
    {
        m_slideArmMotor.setTargetPosition(slide);
        this.m_slideArmMotor.setPower(0.75);
        this.m_slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void slideFudgeFactor(int pull)
    {
        this.m_slideArmMotor.setTargetPosition(this.m_slideArmMotor.getCurrentPosition() + pull);
        this.m_slideArmMotor.setPower(0.75);
        this.m_slideArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean atTarget(double target)
    {
        return this.m_slideArmMotor.getCurrentPosition() <= target+5 && this.m_slideArmMotor.getCurrentPosition() >= target-5;
    }

    public void stop()
    {
        this.m_slideArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
