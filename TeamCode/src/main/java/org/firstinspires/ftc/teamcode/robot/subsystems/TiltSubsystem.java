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
    private static int targetPosition = 0;

    private static double KP = 0.0012, KI = 0.0, kD = 0.0000001, KF = 0.2;
    private static double TICKS_IN_DEGREE = 1425.1/360.0;

    private static double TOLERANCE = 4;

    private static double VERTICAL_ENCODER_VALUE = -900;
    // vertical position of tilt when encoders are reset in the starting position

    private PIDController pid = new PIDController(KP, KI, kD);

    private DcMotorEx tilt_motor;

    public TiltSubsystem(HardwareMap hMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        tilt_motor =  hMap.get(DcMotorEx.class, "tilt");

        tilt_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tilt_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tilt_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void init(){ tilt_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tilt_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tilt_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);}

    public void setTargetPosition(int targetPosition)
    {
        this.targetPosition=targetPosition;
    }

    public boolean atTargetPosition()
    {
        if(abs(tilt_motor.getCurrentPosition()-targetPosition)>25) return false;
        return true;
    }

    @Override
    //always chases target position
    public void periodic() {
        double output = 0;
        int currentPos = tilt_motor.getCurrentPosition();
        if(abs(tilt_motor.getCurrentPosition()-targetPosition)>TOLERANCE)
        {
            double pids = pid.calculate(currentPos, targetPosition);
            double ff=0;
            if(currentPos<550) {
                ff = KF * Math.cos(Math.toRadians((currentPos - VERTICAL_ENCODER_VALUE) / TICKS_IN_DEGREE));
                output = pids + ff;
            }else output =pids;

        }
        tilt_motor.setPower(output);
        telemetry.addData("power ", output);
        telemetry.addData("ff: ", KF * Math.cos(Math.toRadians((currentPos-VERTICAL_ENCODER_VALUE)/TICKS_IN_DEGREE)));
        telemetry.addData("pid: ", pid.calculate(currentPos, targetPosition));
        callTelemetry();
    }

    public void callTelemetry()
    {
        telemetry.addData("Tilt Position: ", tilt_motor.getCurrentPosition());
        telemetry.addData("Tilt Target: ", targetPosition);
    }
}