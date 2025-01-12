package teamCode.autoSubsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoDriveSubsystem extends SubsystemBase
{
    private DcMotor m_fLMotor;
    private DcMotor m_fRMotor;
    private DcMotor m_bLMotor;
    private DcMotor m_bRMotor;

    public AutoDriveSubsystem(DcMotor fL, DcMotor fR, DcMotor bL, DcMotor bR)
    {
        this.m_fLMotor = fL;
        this.m_fRMotor = fR;
        this.m_bLMotor = bL;
        this.m_bRMotor = bR;
    }

    public void driveRobot(int fL, int fR, int bL, int bR)
    {
//        this.m_fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.m_fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.m_bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.m_bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.m_fLMotor.setTargetPosition(fL);
        this.m_fRMotor.setTargetPosition(fR);
        this.m_bLMotor.setTargetPosition(bL);
        this.m_bRMotor.setTargetPosition(bR);

        this.m_fLMotor.setPower(0.5);
        this.m_fRMotor.setPower(0.5);
        this.m_bLMotor.setPower(0.5);
        this.m_bRMotor.setPower(0.5);

        this.m_fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.m_fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.m_bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.m_bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stop()
    {
        this.m_fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m_fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m_bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m_bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean atTarget(int target)
    {
        return Math.abs(this.m_fLMotor.getCurrentPosition()) >= target -1;
    }
}