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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TiltSubsystem extends SubsystemBase
{
    Telemetry telemetry;
    private double targetAngle = 0;
    //target angle from horizontal (intake position) in degrees

    //increase this value to increase the speed (and decrease the accuracy / increase inertia) of the tilt
    private static double PID_SPEED = 1;
    private static double KP = 0.004, KI = 0.0, kD = 0.0008;
    private static double KF = 0.3;
    private static double extensionConstant = 0.3;
    private static double TICKS_IN_DEGREE = (1.75*1425.1)/360.0;
    private static double TOLERANCE_PID = 10;
    // tolerance where pid is calculated in ticks
    private static double ACCEPTABLE_POSITION_TOLERANCE_DEGREES = 5;
    // acceptable position tolerance in degrees
    private static int HORIZONTAL_ENCODER_VALUE = 51;
    // horizontal position of tilt when encoders are reset in the starting position

    private PIDController pid = new PIDController(KP, KI, kD);

    private DcMotorEx tilt_motor;

    public TiltSubsystem(HardwareMap hMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        tilt_motor =  hMap.get(DcMotorEx.class, "tilt");
        pid.setTolerance(TOLERANCE_PID);

        tilt_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tilt_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tilt_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        tilt_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetAngle(double targetAngle)
    {
        this.targetAngle = targetAngle;
    }

    public boolean atTargetPosition()
    {
        return abs(targetAngle - toAngle(tilt_motor.getCurrentPosition())) < ACCEPTABLE_POSITION_TOLERANCE_DEGREES;
    }

    private int toEncoder(double angle)
    {
           return (int) (angle * TICKS_IN_DEGREE + HORIZONTAL_ENCODER_VALUE);
    }
    private double toAngle(int encoderValue) // angle where vertical is 0
    {
        return (encoderValue - HORIZONTAL_ENCODER_VALUE)/TICKS_IN_DEGREE;
    }
    private double toAngleFeedforward(int encoderValue) // angle where 0 is the lift starting position
    {
        return (encoderValue/TICKS_IN_DEGREE);
    }
    @Override
    //always chases target position
    public void periodic() {
        double output = 0;
        int currentPos = tilt_motor.getCurrentPosition();
        // current position relative to the arm starting position

        double currentAngle = toAngleFeedforward(currentPos);
        // current angle relative to the arm starting position

        int targetPosition = toEncoder(targetAngle);
        // target position relative to the arm starting position

        // always compensates for gravity
        double ffOutput = KF*Math.cos(Math.toRadians(currentAngle)) *
                (extensionConstant*ExtensionSubsystem.getCurrentPosition());
        double pidOutput = 0;

        //calculates pid if not at target position
        if(!pid.atSetPoint()) pidOutput = this.pid.calculate(currentPos, targetPosition)*PID_SPEED;
        output = ffOutput + pidOutput;

        telemetry.addData("ff: ", ffOutput);
        telemetry.addData("pid: ", pidOutput);
        telemetry.addData("Arm Angle: ", currentAngle);
        telemetry.addData("- output: ", output);
        telemetry.addData("radians", Math.toRadians(currentAngle));

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