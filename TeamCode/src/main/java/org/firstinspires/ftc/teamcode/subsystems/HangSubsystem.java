package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HangSubsystem {
    DcMotor hangMotor;
    public HangSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        hangMotor = hardwareMap.dcMotor.get("hang");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    @SuppressWarnings("unused")
    public void handleMovement(Gamepad gamepad1, Gamepad gamepad2) {
        hangMotor.setPower(-gamepad2.left_stick_y);
    }
}
