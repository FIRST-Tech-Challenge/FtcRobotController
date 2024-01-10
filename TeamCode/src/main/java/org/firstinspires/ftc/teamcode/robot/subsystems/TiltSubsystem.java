package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TiltSubsystem extends SubsystemBase
{
    Telemetry telemetry;
    private double targetAngle = 0;
    //target angle from vertical with positive angles being towards the front of the robot (deposit) and negative towards the back (intake)

    private static double KP = 0.0, KI = 0.0, kD = 0.0;
    private static double KF = -0.2;
    private static double TICKS_IN_DEGREE = (1.75*1425.1)/360.0;
    private static double TOLERANCE = 0;
    private static double VERTICAL_ENCODER_VALUE = -555;
    // vertical position of tilt when encoders are reset in the starting position

    private PIDController pid = new PIDController(KP, KI, kD);

    private DcMotorEx tilt_motor;

    public TiltSubsystem(HardwareMap hMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        tilt_motor =  hMap.get(DcMotorEx.class, "tilt");
        pid.setTolerance(TOLERANCE);

        tilt_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tilt_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tilt_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init(){tilt_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    tilt_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    tilt_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
    public void setTargetAngle(double targetAngle)
    {
        this.targetAngle = targetAngle;
    }

    public boolean atTargetPosition()
    {
        return pid.atSetPoint();
    }

    private int toEncoder(double angle)
    {
           return (int) (angle * TICKS_IN_DEGREE + VERTICAL_ENCODER_VALUE);
    }
    private double toAngle(int encoderValue)
    {
        return (encoderValue - VERTICAL_ENCODER_VALUE)/TICKS_IN_DEGREE;
    }
    @Override
    //always chases target position
    public void periodic() {
        double output = 0;
        int currentPos = tilt_motor.getCurrentPosition();
        // current position relative to the arm starting position
        double currentAngle = toAngle(currentPos);
        // current angle relative to the vertical

        int targetPosition = toEncoder(targetAngle);
        // target position relative to the arm starting position
        if(!atTargetPosition())
        {
            double pid = this.pid.calculate(currentPos, targetPosition);
            //double ff = KF * Math.cos(Math.toRadians((currentPos-VERTICAL_ENCODER_VALUE)/TICKS_IN_DEGREE) * ExtensionSubsystem.ARM_LENGTH+ExtensionSubsystem.UNEXTENDED_POSITION+ExtensionSubsystem.getCurrentPosition());
            double ff = KF * Math.cos(Math.toRadians(currentAngle));
            output = pid + ff;

            telemetry.addData("ff: ", ff);
            telemetry.addData("pid: ", pid);
            telemetry.addData("Arm Angle: ", currentAngle);
            telemetry.addData("- output: ", output);
        }
        tilt_motor.setPower(output);
        callTelemetry();
    }

    private void callTelemetry()
    {
        telemetry.addData("- power ", tilt_motor.getPower());
        telemetry.addData("Encoder Position: ", tilt_motor.getCurrentPosition());
        telemetry.addData("PID Target: ", pid.getSetPoint());
    }
}