package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class TiltSubsystem extends SubsystemBase
{
    private static int targetPosition = 0;

    private static double kS = 0.0, kCos = 0.0, kV = 0.0, kA = 0.0;
    private static double velocity = 2;
    private static double acceleration = 3;

    private static double TOLERANCE = 10;

    private ArmFeedforward feedforward;

    private DcMotorEx tilt_motor;

    public TiltSubsystem(HardwareMap hMap)
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tilt_motor =  hMap.get(DcMotorEx.class, "tilt");
        ArmFeedforward feedforward = new ArmFeedforward(kS, kCos, kV, kA);

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
        if(abs(tilt_motor.getCurrentPosition()-targetPosition)>TOLERANCE)
            output = feedforward.calculate(targetPosition, 2,3);
        tilt_motor.setVelocity(output);
    }
}
