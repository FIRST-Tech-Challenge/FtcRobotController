package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeAndWristSubsystem {
    private final CRServo intake;
    private final Servo wrist;
    Telemetry telemetry;

    public final double INTAKE_COLLECT = -1.0;
    public final double INTAKE_OFF = 0.0;
    public final double INTAKE_DEPOSIT = 0.5;
    public final double WRIST_FOLDED_IN = 0.2;
    public final double WRIST_FOLDED_OUT = 0.5;

    private double intakePower = INTAKE_OFF;
    private double wristPosition = 0.4;
    public IntakeAndWristSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = hardwareMap.crservo.get("IN");
        wrist = hardwareMap.servo.get("WR");

        intake.setPower(INTAKE_OFF);

        wrist.setPosition(wristPosition);
    }
    public void handleMovementAutonomous(double csServo, double Servo) {
        intake.setPower(csServo);
        wrist.setPosition(Servo);
    }
    @SuppressWarnings("unused")
    public void handleMovementTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        readControls(gamepad2);

        intake.setPower(intakePower);
        wrist.setPosition(wristPosition);
    }
    public void updateTelemetry() {
         telemetry.addData("Intake Power", intake.getPower());
         telemetry.addData("Wrist Position", wrist.getPosition());
    }
    private void readControls(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            intakePower = INTAKE_COLLECT;
        } else if (gamepad.left_bumper) {
            intakePower = INTAKE_DEPOSIT;
        } else {
            intakePower = INTAKE_OFF;
        }

    }
}
