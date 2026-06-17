package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CandyCane;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "Bomber", group = "TeleOp")
public class BomberTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        CandyCane candyCane = new CandyCane(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
            candyCane.setPower(gamepad1.left_trigger);
        }

        drivetrain.stop();
        candyCane.stop();
    }
}
