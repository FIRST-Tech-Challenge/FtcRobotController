  package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.Util.Constants.*;
import static org.firstinspires.ftc.teamcode.Util.IDs.*;

  @TeleOp
public class Robot extends OpMode {

    org.firstinspires.ftc.teamcode.Subsystem.Drivetrain drivetrain = new org.firstinspires.ftc.teamcode.Subsystem.Drivetrain();

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
    this.drivetrain.mechanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

    this.shoulder.runArmMotor(gamepad2.left_stick_y);
        
    this.slides.slidePower(gamepad2.right_stick_y);
        
    if (gamepad2.a) {
            claw.setServoClawPos(OPEN_CLAW_VALUE);
        } else {
            claw.setServoClawPos(CLOSED_CLAW_VAULE);
        }

        if (gamepad2.b) {
            wrist.setServoWristPos(OPEN_WRIST_VALUE);
        } else {
            wrist.setServoWristPos(CLOSED_WRIST_VALUE);
        }
    }
}

