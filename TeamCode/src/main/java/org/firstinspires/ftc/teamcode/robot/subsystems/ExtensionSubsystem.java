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
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ExtensionSubsystem extends SubsystemBase
{
    public static final int UNEXTENDED_POSITION = 0;
    public static final int BACKBOARD_POSITION_INCREMENT = 20;

    private static double kP = 0.0, kI = 0.0, kD = 0.0, kF = 0.0;

    public static double TOLERANCE = 10;

    private PIDFController pidf;

    private static Motor extension_top;
    private static Motor extension_bottom;
    private static MotorGroup extension;

    public ExtensionSubsystem(HardwareMap hMap)
    {
        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setTolerance(TOLERANCE);
        extension_top =  new Motor(hMap, "extension_motor_1");
        extension_bottom =  new Motor(hMap, "extension_motor_2");
        extension = new MotorGroup(extension_top, extension_bottom);

      //  telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        extension.resetEncoder(); // RESET_ENCODERS
        extension.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extension.setRunMode(Motor.RunMode.RawPower);
    }

    /**
     * asynchronous pidf goToPosition
     *
     * @param targetPosition Target Position
     */
    public void goToPosition(int targetPosition)
    {
        pidf.setSetPoint(targetPosition);
        if(!pidf.atSetPoint())extension.set(pidf.calculate(extension.getCurrentPosition(), targetPosition));
        else extension.set(0);
    }

    public boolean atTargetPosition ()
    {
        return pidf.atSetPoint();
    }

    /**
     * Power control through joystick
     *
     * @param joystick value of the joystick from -1 to 1
     */
    public void manualControl(double joystick)
    {
        if(extension.getCurrentPosition()>UNEXTENDED_POSITION)extension.set(joystick);
    }
}
