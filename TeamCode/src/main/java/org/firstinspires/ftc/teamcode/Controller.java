package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "controller movement", group = "SA_FTC")
public class Controller extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        MovementUtils movementUtils = new MovementUtils(hardwareMap);
        ArmUtils armUtils = new ArmUtils(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        armUtils.startupSequence();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            movementUtils.movement(gamepad1);
            armUtils.roller(gamepad2);
            armUtils.extend(gamepad2);
            armUtils.lift(gamepad2);
            armUtils.grip(gamepad2);
            armUtils.runSequences(gamepad2);

            // Debugging
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
