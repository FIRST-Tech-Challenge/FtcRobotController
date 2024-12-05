  package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.constants.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Robot extends OpMode {

    Drivetrain drivetrain = new Drivetrain();

    Shoulder shoulder = new Shoulder();
    Slides slides = new Slides();

    Claw claw = new Claw();
    Wrist wrist = new Wrist();

    @Override
    public void init() {
        drivetrain.init(hardwareMap);

        this.shoulder.init(hardwareMap);
        this.slides.init(hardwareMap);

        claw.init(hardwareMap);
        wrist.init(hardwareMap);

    }

    @Override
    public void loop() {
        
    drivetrain.loop(gamepad1.left_stick_y,gamepad1.right_stick_x, gamepad1.left_stick_x);
        
    this.shoulder.runArmMotor(gamepad1.left_stick_y);
        
    this.slides.slidePower(gamepad1.right_stick_y);
        
    if (gamepad1.a) {
            claw.setServoClawPos(openClawValue);
        } else {
            claw.setServoClawPos(closedClawValue);
        }

        if (gamepad1.b) {
            wrist.setServoWristPos(openWristValue);
        } else {
            wrist.setServoWristPos(closedWristValue);
        }
    }
}

