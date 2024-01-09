package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TiltSubsystem extends SubsystemBase
{
    Telemetry telemetry;
    private static int targetPosition = 0;

    private static double KP = 0.0, KI = 0.0, kD = 0.0, KF = 0.0;
    private static double TICKS_IN_DEGREE = 700/180.0;

    private static double TOLERANCE = 10;

    private PIDController pid = new PIDController(KP, KI, kD);

    private DcMotorEx tilt_motor;

    public TiltSubsystem(HardwareMap hMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        tilt_motor =  hMap.get(DcMotorEx.class, "tilt");

        tilt_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetPosition(int targetPosition)
    {
        this.targetPosition=targetPosition;
    }

    public boolean atTargetPosition()
    {
        if(abs(tilt_motor.getCurrentPosition()-targetPosition)>TOLERANCE) return false;
        return true;
    }

    @Override
    //always chases target position
    public void periodic() {
        double output = 0;
        int currentPos = tilt_motor.getCurrentPosition();
        if(abs(currentPos-targetPosition)>TOLERANCE)
        {
            double pid = this.pid.calculate(currentPos, targetPosition);
            double ff_result = KF * Math.cos(Math.toRadians(currentPos/TICKS_IN_DEGREE));
            output = pid + ff_result;
        }
        tilt_motor.setVelocity(output);
        callTelemetry();
    }

    public void callTelemetry()
    {
        telemetry.addData("Tilt Position: ", tilt_motor.getCurrentPosition());
        telemetry.addData("Tilt Target: ", targetPosition);
    }
}
