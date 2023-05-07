package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.classes.PIDOpenClosed;

public class Drivetrain extends SubsystemBase {

    //DRIVE MOTORS
    private DcMotor m_fL;
    private DcMotor m_fR;
    private DcMotor m_rL;
    private DcMotor m_rR;

    //DRIVE VARIABLES
    private boolean fieldCentric = false;
    private double totalSpeed = 0.5;

    //TURNING VARIABLES
    private PIDCoefficientsEx turningCoeffs;
    private PIDEx turningPID;
    private AngleController turningController;
    private PIDOpenClosed turnPID;

    private MecanumDrive drive = null;
    private RevIMU imu = null;

    public Drivetrain(final HardwareMap hwMap) {
        Motor fL = new Motor(hwMap, "fL", Motor.GoBILDA.RPM_312);
        Motor fR = new Motor(hwMap, "fR", Motor.GoBILDA.RPM_312);
        Motor rL = new Motor(hwMap, "rL", Motor.GoBILDA.RPM_312);
        Motor rR = new Motor(hwMap, "rR", Motor.GoBILDA.RPM_312);

        m_fL = fL.motor;
        m_fR = fR.motor;
        m_rL = rL.motor;
        m_rR = rR.motor;

        m_fL.setDirection(DcMotorSimple.Direction.REVERSE);
        m_rL.setDirection(DcMotorSimple.Direction.REVERSE);

        m_fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(fL, fR, rL, rR);

        //TURNING PID
        turningCoeffs = new PIDCoefficientsEx(2.5, 0.4, .4, 0.25, 2, 0.5);
        turningPID = new PIDEx(turningCoeffs);
        turningController = new AngleController(turningPID);
        turnPID = new PIDOpenClosed(turningController, 0.2);

        //GYRO
        imu = new RevIMU(hwMap);
        imu.init();
    }

    public void robotGoSkrtSkrt(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        strafeSpeed *= totalSpeed;
        forwardSpeed *= totalSpeed;
        turnSpeed *= totalSpeed;
        if (fieldCentric) {
            drive.driveFieldCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    imu.getRotation2d().getDegrees()
            );
        } else {
            drive.driveRobotCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed
            );
        }
    }
}
