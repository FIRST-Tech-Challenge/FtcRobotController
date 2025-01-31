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

    public final double ARM_COLLECT = 0;
    public final double ARM_CLEAR_BARRIER = 25;
    public final double ARM_SCORE_SAMPLE_IN_LOW = 90;
    public final double ARM_MAX_HEIGHT = 125;
    public final double ARM_SCORE_SAMPLE_IN_HIGH = 120;
    double comp = 0;
    double armPosition = ARM_COLLECT;

    private final DcMotor armMotor;
    private final DcMotorEx armMotorEx;
    @SuppressWarnings("unused")
    public void handleMovementTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        readControls(gamepad1, gamepad2);
        setPosition();
    }
    private void calculateCompensation() {
        final double height = 200;
        final double slideLength = 400;
        double normalizedSlidePosition = Common.slidePosition;
        comp = Math.toDegrees(Math.acos(height/(slideLength + normalizedSlidePosition)) - Math.acos(height/ slideLength))/1.3 + 2;
        telemetry.addData("Compensation", comp);
    }
    private void setPosition() {
        calculateCompensation();
        int pos = (int) Common.degreesToTicks(armPosition);
        telemetry.addData("calculated position", pos);
        armMotor.setTargetPosition(Math.max(pos, (int) Common.degreesToTicks(comp)));
        if (armMotorEx != null) {
            armMotorEx.setVelocity(1500 - armMotor.getCurrentPosition()*0.3);
            Common.warnIfOvercurrent(armMotorEx, telemetry, "ArmSubsystem");
        }
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setPositionAuto(double pos) {
        armPosition = Math.max(Math.min(pos, ARM_MAX_HEIGHT), 0);
        setPosition();
    }

    public void updateTelemetry() {
        Common.armPosition = armMotor.getCurrentPosition();
        telemetry.addData("ArmSubsystem Target Position", armMotor.getTargetPosition());
        telemetry.addData("ArmSubsystem Encoder", armMotor.getCurrentPosition());
        if (armMotorEx != null) telemetry.addData("ArmSubsystem Current", armMotorEx.getCurrent(CurrentUnit.AMPS));
    }
    private void readControls(Gamepad gamepad1, Gamepad gamepad2) {
        armPosition = Math.max(Math.min(armPosition + (50*Common.cycleTime * -gamepad2.right_stick_y), ARM_MAX_HEIGHT), 0);
        if (gamepad2.a) armPosition = ARM_COLLECT;
        else if (gamepad2.b) armPosition = ARM_CLEAR_BARRIER;
        else if (gamepad2.x) armPosition = ARM_SCORE_SAMPLE_IN_LOW;
        else if (gamepad2.y) armPosition = ARM_SCORE_SAMPLE_IN_HIGH;

    }

    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // init motor
        armMotor = hardwareMap.dcMotor.get("AM");
        this.telemetry = telemetry;
        armMotorEx = Common.convertToDcMotorExOrWarn(armMotor, telemetry, "ArmSubsystem").orElse(null);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
