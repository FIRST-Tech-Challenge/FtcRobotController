package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.Common;

public class ArmSubsystem {
    Telemetry telemetry;

    final double ARM_COLLECT = 0;
    final double ARM_CLEAR_BARRIER = 15;
    final double ARM_SCORE_SPECIMEN = 90;
    final double FUDGE_FACTOR = 15.0;
    final double ARM_SCORE_SAMPLE_IN_LOW = 90;
    final double ARM_ATTACH_HANGING_HOOK = 110;
    final double ARM_LIFT_COMPENSATION_FACTOR = 0.01291803147;
    double armPosition = ARM_COLLECT;
    double armPositionFudgeFactor;

    private final DcMotor armMotor;
    private final DcMotorEx armMotorEx;

    public void handleMovementTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        readControls(gamepad1);

        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

        setPosition();
    }

    private void setPosition() {
        armMotor.setTargetPosition((int) Common.degreesToTicks(armPosition + armPositionFudgeFactor));
        if (armMotorEx != null) {
            armMotorEx.setVelocity(2100);
            Common.warnIfOvercurrent(armMotorEx, telemetry, "ArmSubsystem");
        }
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void updateArmLiftCompensation(double liftPosition) {
        double armLiftComp;
        if (armPosition < Common.degreesToTicks(45)) {
            armLiftComp = ARM_LIFT_COMPENSATION_FACTOR * liftPosition;
        } else {
            armLiftComp = 0;
        }
        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
    }
    public void updateTelemetry() {
        telemetry.addData("ArmSubsystem Target Position", armMotor.getTargetPosition());
        telemetry.addData("ArmSubsystem Encoder", armMotor.getCurrentPosition());
        if (armMotorEx != null) telemetry.addData("ArmSubsystem Current", armMotorEx.getCurrent(CurrentUnit.AMPS));
    }
    private void readControls(Gamepad gamepad1) {
        if (gamepad1.a) armPosition = ARM_COLLECT;
        else if (gamepad1.b) armPosition = ARM_CLEAR_BARRIER;
        else if (gamepad1.x) armPosition = ARM_SCORE_SAMPLE_IN_LOW;
        else if (gamepad1.dpad_left) armPosition = ARM_COLLECT;
        else if (gamepad1.dpad_right) armPosition = ARM_ATTACH_HANGING_HOOK;
        else if (gamepad1.dpad_up) armPosition = ARM_ATTACH_HANGING_HOOK;
        else if (gamepad1.dpad_down) armPosition = ARM_SCORE_SPECIMEN;
    }

    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // init motor
        armMotor = hardwareMap.dcMotor.get("AM");
        this.telemetry = telemetry;
        armMotorEx = Common.convertToDcMotorExOrWarn(armMotor, telemetry, "ArmSubsystem").orElse(null);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
