package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.Constants;

import java.util.concurrent.TimeUnit;

public class IntakeSubsystem {
    private DcMotor intake;
    private double intakePower;

    private ElapsedTime runtime;

    public IntakeSubsystem(DcMotor intake, ElapsedTime runtime) {
        this.intake = intake;
        this.runtime = runtime;
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

    public enum Modes {
        INTAKE,
        OUTTAKE
    }
}
