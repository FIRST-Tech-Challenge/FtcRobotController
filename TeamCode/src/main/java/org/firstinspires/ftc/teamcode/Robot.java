package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Robot extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        
    drivetrain.loop(gamepad1.left_stick_y,gamepad1.right_stick_x, gamepad1.left_stick_x);
        
    this.arm.runArmMotor(gamepad1.left_stick_y);
        
    this.linerarSlides.slidePower(gamepad1.right_stick_y);
        
    if (gamepad1.a) {
            Claw.setServoClawPos(openClawValue);
        } else {
            Claw.setServoClawPos(closedClawValue);
        }

        if (gamepad1.b) {
            Wrist.setServoWristPos(openWristValue);
        } else {
            Wrist.setServoWristPos(closedWristValue);
        }
    }
}
