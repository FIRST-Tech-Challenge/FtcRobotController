package org.firstinspires.ftc.teamcode.opmodes.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlidePreset {
    private DcMotor slideMotor;

    // Define preset positions (in encoder ticks)
    private final int GROUND_POSITION = 0;
    private final int LOW_POSITION = 500;
    private final int MID_POSITION = 1000;
    private final int HIGH_POSITION = 1500;

    public SlidePreset(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, "armMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void controlSlide(Gamepad gamepad) {
        /*
        if (gamepad.a) {
            moveToPosition(GROUND_POSITION);
        } else if (gamepad.b) {
            moveToPosition(LOW_POSITION);
        } else if (gamepad.x) {
            moveToPosition(MID_POSITION);
        } else if (gamepad.y) {
            moveToPosition(HIGH_POSITION);
        } else*/ if (gamepad.dpad_right) {
            moveToPosition(MID_POSITION);
        }
    }

    private void moveToPosition(int targetPosition) {
        slideMotor.setTargetPosition(targetPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1.0);

        while (slideMotor.isBusy()) {
            // Optionally, add telemetry updates in the main TeleOp loop
        }

        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
