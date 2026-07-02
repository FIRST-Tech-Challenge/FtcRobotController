package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CandyCane;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Bomber", group = "TeleOp")
public class BomberTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        CandyCane candyCane = new CandyCane(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
            candyCane.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            intake.setPower((gamepad1.left_bumper ? 1 : 0) - (gamepad1.right_bumper ? 1 : 0));
        }

        drivetrain.stop();
        candyCane.stop();
        intake.stop();
    }
}
