package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.commands.extension.ExtensionGoToPosition;

@Config
public class ExtensionSubsystem extends SubsystemBase
{
    Telemetry telemetry;

    public static final int UNEXTENDED_POSITION = 0;
    public static final int BACKBOARD_POSITION_INCREMENT = 20;

    private static double kP = 0.0, kI = 0.0, kD = 0.0, kF = 0.0;

    public static double TOLERANCE_PID = 10;
    // tolerance where pid is calculated in ticks
    public static double ACCEPTABLE_POSITION_TOLERANCE = 20;
    // acceptable position tolerance is the tolerance for the position to be considered "at position"

    private PIDFController pidf;

    private static Motor extension_top;
    private static Motor extension_bottom;
    private static MotorGroup extension;

    public ExtensionSubsystem(HardwareMap hMap, Telemetry telemetry)
    {
        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setTolerance(TOLERANCE_PID);
        extension_top =  new Motor(hMap, "extension_motor_1");
        extension_bottom =  new Motor(hMap, "extension_motor_2");
        extension = new MotorGroup(extension_top, extension_bottom);

        this.telemetry = telemetry;

        extension.resetEncoder(); // RESET_ENCODERS
        extension.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extension.setRunMode(Motor.RunMode.RawPower);
    }

    public static int getCurrentPosition() {
        return extension.getCurrentPosition();
    }

    /**
     * asynchronous pidf goToPosition
     *
     * @param targetPosition Target Position
     */
    public void goToPosition(int targetPosition)
    {
        pidf.setSetPoint(targetPosition);
        if(!atTargetPosition())extension.set(pidf.calculate(extension.getCurrentPosition(), targetPosition));
        else extension.set(0);
    }
    // TODO: potential problem with not tracking the position of the
    //  extension whenever the motor isn't actively trying to go to a spot
    public boolean atTargetPosition ()
    {
        return abs(pidf.getSetPoint() - extension.getCurrentPosition()) < ACCEPTABLE_POSITION_TOLERANCE;
    }

    /**
     * Power control through joystick
     *
     * @param joystick value of the joystick from -1 to 1
     */
    public void manualControl(double joystick)
    {
        if(extension.getCurrentPosition()>UNEXTENDED_POSITION&&extension.getCurrentPosition()<ExtensionGoToPosition.ONE_STAGE_EXTENSION)extension.set(joystick);
    }

    public void periodic()
    {
       callTelemetry();
    }

    public void callTelemetry()
    {
        telemetry.addData("Extension Position", extension.getCurrentPosition());
        telemetry.addData("Extension Target Position", pidf.getSetPoint());
    }
}
