package org.firstinspires.ftc.teamcode.Test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class IntakeTest extends LinearOpMode {

    private MotorEx intakeMotor;
    public static double p = 0.5, i = 0, d = 0;
    private double currentVelocity;
    public static double targetVelocity;
    public static final double TICKS_PER_ROT = 145.1;
    public static double kS = 0, kV = 1.055, kA = 0;


    @Override
    public void runOpMode() {
        intakeMotor = new MotorEx(hardwareMap, "IM", Motor.GoBILDA.RPM_1150);
        intakeMotor.setRunMode(Motor.RunMode.VelocityControl);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {

            currentVelocity = intakeMotor.getVelocity();
            intakeMotor.setFeedforwardCoefficients(kS, kV, kA);
            intakeMotor.setVeloCoefficients(p, i, d);
            intakeMotor.setVelocity(targetVelocity);

            telemetry.addData("Current Velocity (rot/sec): ", currentVelocity);
            telemetry.addData("Target Velocity (rot/sec): ", targetVelocity);
            telemetry.update();
        }
    }
}