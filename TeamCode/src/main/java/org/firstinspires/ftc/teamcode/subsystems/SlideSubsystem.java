package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.Common;

public class SlideSubsystem {
    private final DcMotor slideMotor;
    private final DcMotorEx slideMotorEx;
    private final Telemetry telemetry;

    public final double SLIDE_COLLAPSED = 0.0;
    boolean holdingReset = false;
    // TODO: Change this to the correct value
//    public final double SLIDE_SCORING_IN_LOW_BASKET = 0.0;
    public final double SLIDE_SCORING_IN_HIGH_BASKET = 540.0;
    double slidePosition = SLIDE_COLLAPSED;

    public SlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        slideMotor = hardwareMap.dcMotor.get("SM");
        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        slideMotorEx = Common.convertToDcMotorExOrWarn(slideMotor, telemetry, "SlideSubsystem Motor").orElse(null);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        this.telemetry = telemetry;
    }


    private void readControls(Gamepad gamepad1, Gamepad gamepad2) {
        double cycleTime = Common.cycleTime;
        if (gamepad2.right_trigger > 0.5) {
            slidePosition += 850 * cycleTime * gamepad2.right_trigger * 2;
        } else if (gamepad2.left_trigger > 0.5) {
            slidePosition -= 850 * cycleTime * gamepad2.left_trigger * 2;
        }

        if (gamepad2.back) {
            holdingReset = true;
        }
        if (holdingReset) {
            slidePosition -= 1500*Common.cycleTime;
        } else {
            slidePosition = Common.clamp(slidePosition, SLIDE_COLLAPSED, SLIDE_SCORING_IN_HIGH_BASKET);
        }
        if (gamepad2.start) {
            holdingReset = false;
            slidePosition = 0;
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    @SuppressWarnings("unused")
    public void handleMovementTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        readControls(gamepad1, gamepad2);
        setPosition();
    }

    public void updateTelemetry() {
        Common.slidePosition = slideMotor.getCurrentPosition();
        telemetry.addData("SlideSubsystem Target Position", slideMotor.getTargetPosition());
        telemetry.addData("SlideSubsystem Encoder", slideMotor.getCurrentPosition());
        if (slideMotorEx != null) telemetry.addData("SlideSubsystem Current", slideMotorEx.getCurrent(CurrentUnit.AMPS));
    }
    public void handleMovementAuto(double pos) {
        slidePosition = pos;
        setPosition();
    }

    private void setPosition() {
        slideMotor.setTargetPosition((int) Common.slideServoMMToTicks(slidePosition));
        if (slideMotorEx != null) {
            slideMotorEx.setVelocity(2100);
            Common.warnIfOvercurrent(slideMotorEx, telemetry, "SlideSubsystem");
        }
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}
