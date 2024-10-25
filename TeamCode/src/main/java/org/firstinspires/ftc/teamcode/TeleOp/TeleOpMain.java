package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

@TeleOp
@Config
public class TeleOpMain extends LinearOpMode {
    Gamepad DriverTwo = gamepad2;
    Gamepad DriverOne = gamepad1;
    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drive = new Drivetrain(this);
        Arm arm = new Arm(this);
        Slides slides = new Slides(this);
        Claw claw = new Claw(this);
        Wrist wrist = new Wrist(this);

        waitForStart();
        
        drive.teleOp();
        arm.teleOp();
        slides.teleOp(DriverOne);
        claw.teleOp();
        wrist.teleOp();
    }
}
