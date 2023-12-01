package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

import java.util.concurrent.TimeUnit;

public class IntakeSubsystem {
    private DcMotor intake;
    private double intakePower;

    private ElapsedTime runtime;
    Telemetry telemetry;

    public enum Modes {
        INTAKE,
        OUTTAKE
    }

    public IntakeSubsystem(DcMotor intake, ElapsedTime runtime, Telemetry telemetry) {
        this.intake = intake;
        this.runtime = runtime;
        this.telemetry = telemetry;
        initialize();
    }

    public void initialize() {
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void teleOPIntake(double gamepad2RightTrigger, double gamepad2LeftTrigger) {
        intakePower = gamepad2RightTrigger - gamepad2LeftTrigger;
        intake.setPower(intakePower);
    }

    public void intake() {
        intake.setPower(Constants.autoIntakePower);
    }

    public void outtake() {
        intake.setPower(-Constants.autoIntakePower);
    }

    public void stopMotor() {
        intake.setPower(0.0);
    }

    public void autoIntake(Modes mode, double timeSeconds) {
        double startingTime = runtime.time(TimeUnit.SECONDS);
        while (runtime.time(TimeUnit.SECONDS) <= startingTime + timeSeconds) {
            switch (mode) {
                case INTAKE:
                    intake.setPower(-Constants.autoIntakePower);
                    break;
                case OUTTAKE:
                    intake.setPower(Constants.autoIntakePower);
                    break;
                default:
                    break;
            }
        }

        intakePower = 0.0;
        intake.setPower(intakePower);
    }
}
