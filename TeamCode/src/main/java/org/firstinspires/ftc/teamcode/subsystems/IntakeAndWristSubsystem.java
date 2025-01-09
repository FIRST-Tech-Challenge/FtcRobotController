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

    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;
    final double WRIST_FOLDED_IN = 1.0 / 6.0;
    final double WRIST_FOLDED_OUT = 0.5;

    private double intakePower = INTAKE_OFF;
    private double wristPosition = WRIST_FOLDED_IN;
    public IntakeAndWristSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = hardwareMap.crservo.get("IN");
        wrist = hardwareMap.servo.get("WR");

        intake.setPower(INTAKE_OFF);

        wrist.setPosition(WRIST_FOLDED_IN);
    }

    @SuppressWarnings("unused")
    public void handleMovementTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        readControls(gamepad1);

        intake.setPower(intakePower);
        wrist.setPosition(wristPosition);

    }
    public void updateTelemetry() {
         telemetry.addData("Intake Power", intake.getPower());
         telemetry.addData("Wrist Position", wrist.getPosition());
    }
    private void readControls(Gamepad gamepad1) {
        if (gamepad1.right_bumper) {
            intakePower = INTAKE_COLLECT;
        } else if (gamepad1.left_bumper) {
            intakePower = INTAKE_DEPOSIT;
        } else {
            intakePower = INTAKE_OFF;
        }

        if (gamepad1.dpad_up) {
            wristPosition = WRIST_FOLDED_OUT;
        } else if (gamepad1.dpad_down) {
            wristPosition = WRIST_FOLDED_IN;
        }
    }
}
