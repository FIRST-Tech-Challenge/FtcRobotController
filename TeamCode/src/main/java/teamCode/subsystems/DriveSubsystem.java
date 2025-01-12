package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase
{
    public MecanumDrive m_drive;
    private DcMotor m_fLMotor;
    private DcMotor m_fRMotor;
    private DcMotor m_bLMotor;
    private DcMotor m_bRMotor;

    private int m_fLPos;
    private int m_fRPos;
    private int m_bLPos;
    private int m_bRPos;

    private Orientation m_lastRecordedAngle;
    private double m_currentAngle;
    private double error;

    IMU m_imu;

    public DriveSubsystem(MecanumDrive drive, IMU imu)
    {
        this.m_drive = drive;
        this.m_lastRecordedAngle = new Orientation();
        this.m_currentAngle = 0.0;
        this.m_imu = imu;

    }

    public void headingDrive(double leftX, double leftY, double rightX, double rightY)
    {
        m_drive.driveFieldCentric
                (
                        leftX * leftX * leftX * -1,
                        leftY * leftY * leftY * -1,
                        getTurnPower(rightX, rightY),
                        this.m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
                );
//        System.out.println("Error: " + error);
//        getTurnPower(rightX, rightY);

//        System.out.println("Error: " + error);
//        System.out.println("Error: " + error);
//        System.out.println(this.m_imu.getRobotYawPitchRollAngles().getYaw());
//        System.out.println("Turn angle: " + Math.atan2(rightX, rightY * -1) * -1 * (180 / Math.PI));
    }

    public void driveRobot(int fL, int fR, int bL, int bR)
    {
        m_fLPos += fL;
        m_fRPos += fR;
        m_bLPos += bL;
        m_bRPos += bR;

        m_fLMotor.setTargetPosition(m_fLPos);
        m_fRMotor.setTargetPosition(m_fRPos);
        m_bLMotor.setTargetPosition(m_bLPos);
        m_bRMotor.setTargetPosition(m_bRPos);

        this.m_fLMotor.setPower(0.5);
        this.m_fRMotor.setPower(0.5);
        this.m_bLMotor.setPower(0.5);
        this.m_bRMotor.setPower(0.5);

//        this.m_fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.m_fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.m_bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.m_bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getTurnPower(double rightX, double rightY)
    {
        turnTo(rightX, rightY);
        System.out.println("Running!");

        if (Math.abs(error) > 6)
        {
            double motorPower = 0.5;
            error = error - getAngle();
//            this.m_robot.driveWithMotorPowers(motorPower, -motorPower, motorPower, -motorPower);
//            this.m_drive.driveWithMotorPowers(motorPower, -motorPower, motorPower, -motorPower);
            return motorPower * error / 100 + (0.1 * (error / Math.abs(error)));
        }
        else
        {
            return 0.0;
        }
    }

    public void turnTo(double rightX, double rightY)
    {
        Orientation orientation = m_imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double desiredAngle;
        if (rightX > 0.5 || rightX < -0.5 || rightY > 0.5 || rightY < -0.5)
        {
            desiredAngle = Math.atan2(rightX, rightY * -1) * -1 * (180 / Math.PI);
        }
        else
        {
            desiredAngle = orientation.firstAngle;
        }

        error = desiredAngle - orientation.firstAngle;

        if(error > 180)
        {
            error -= 360;
        }
        else if (error < -180)
        {
            error += 360;
        }

        turn(error, desiredAngle);
    }

    public void turn(double degrees, double desiredAngle)
    {
        resetAngle();

        error = degrees;
    }
    public void resetAngle()
    {
        m_lastRecordedAngle = m_imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, BNO055IMU.AngleUnit.DEGREES.toAngleUnit());
        m_currentAngle = 0;
    }

    public double getAngle()
    {
        Orientation orientation = this.m_imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - m_lastRecordedAngle.firstAngle;

        if (deltaAngle > 180)
        {
            deltaAngle -= 360;
        }
        else if (deltaAngle <= -180)
        {
            deltaAngle += 360;
        }

        m_currentAngle += deltaAngle;
        m_lastRecordedAngle = orientation;
        return m_currentAngle;
    }
}