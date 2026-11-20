package org.firstinspires.ftc.teamcode.zeroCode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.zeroCode.finalClassess.Constants;

@TeleOp(name="powerOneMotor", group="Testing")
public class powerOneMotorTestingTeleOp extends OpMode {
    private String oneMotorName = "oneMotor";
    private DcMotor oneMotor;
    @Override
    public void init() {
        Constants.hardware.resetHardwareMaps(telemetry);
        oneMotor = hardwareMap.get(DcMotor.class, oneMotorName);

    }

    @Override
    public void loop() {
        double power = gamepad1.left_stick_y;
        if (gamepad1.a) {
            power = 1;
        } else if (gamepad1.b) {
            power = -1;
        } else if (gamepad1.x) {
            power = 10;
        } else if (gamepad1.y) {
            power = 100;
        }
        oneMotor.setPower(power);
        telemetry.addData("Set Power", power);
        telemetry.addData("Real Power", oneMotor.getPower());
        telemetry.update();
    }
}
