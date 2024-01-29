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

    private static double KP = 0.004, KI = 0.0, kD = 0.0008,KF =0.3;

    private static double TICKS_IN_DEGREE = 700/180.0;

    private static double TOLERANCE = 10;

    private static double VERTICAL_ENCODER_VALUE = 300;
    // vertical position of tilt when encoders are reset in the starting position

    private PIDController pid = new PIDController(KP, KI, kD);

    private DcMotorEx tilt_motor;

    public TiltSubsystem(HardwareMap hMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        tilt_motor =  hMap.get(DcMotorEx.class, "tilt");

        tilt_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tilt_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetPosition(int targetPosition)
    {
        this.targetPosition=targetPosition;
    }

    public boolean atTargetPosition()
    {
        if(abs(tilt_motor.getCurrentPosition()-targetPosition)>30) return false;
        return true;
    }

    @Override
    //always chases target position
    public void periodic() {
        double output = 0;
        int currentPos = tilt_motor.getCurrentPosition();
        if((abs(tilt_motor.getCurrentPosition()-targetPosition)<5))
        {
            double pid = this.pid.calculate(currentPos, targetPosition);
            double ff = KF * Math.cos(Math.toRadians((currentPos-VERTICAL_ENCODER_VALUE)/TICKS_IN_DEGREE));
            output = pid + ff;
        }
        tilt_motor.setPower(output);
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
