package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LiftSubsystem {
    private final DcMotor liftMotor;
    private final DcMotorEx liftMotorEx;
    private final Telemetry telemetry;

    final double LIFT_COLLAPSED = 0.0;

    // TODO: Change this to the correct value
//    final double LIFT_SCORING_IN_LOW_BASKET = 0.0;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480.0;
    double liftPosition = LIFT_COLLAPSED;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        liftMotorEx = Common.convertToDcMotorExOrWarn(liftMotor, telemetry, "LiftSubsystem Motor").orElse(null);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        this.telemetry = telemetry;
    }


    private void readControls(Gamepad gamepad2) {
        double cycleTime = Common.cycleTime;
        if (gamepad2.right_bumper) {
            liftPosition += 850 * cycleTime;
        } else if (gamepad2.left_bumper) {
            liftPosition -= 850 * cycleTime;
        }
        liftPosition = Common.clamp(liftPosition, LIFT_COLLAPSED, LIFT_SCORING_IN_HIGH_BASKET);
    }
    @SuppressWarnings("unused")
    public void handleMovement(Gamepad gamepad1, Gamepad gamepad2) {
        readControls(gamepad2);
        setPosition();


    }

    public void updateTelemetry() {
        telemetry.addData("LiftSubsystem Target Position", liftMotor.getTargetPosition());
        telemetry.addData("LiftSubsystem Encoder", liftMotor.getCurrentPosition());
        if (liftMotorEx != null) telemetry.addData("LiftSubsystem Current", liftMotorEx.getCurrent(CurrentUnit.AMPS));
    }

    private void setPosition() {
        liftMotor.setTargetPosition((int) Common.mmToTicks(liftPosition));
        if (liftMotorEx != null) {
            liftMotorEx.setVelocity(2100);
            Common.warnIfOvercurrent(liftMotorEx, telemetry, "LiftSubsystem");
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getLiftPosition() {
        return liftPosition;
    }
}
