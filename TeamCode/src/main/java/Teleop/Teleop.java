package Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Constants;

import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter;

@TeleOp
public class Teleop extends LinearOpMode {

    Gamepad activeGamepad1;

    Gamepad previousGamepad1;

    static boolean manualMode;

    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter Shooter;


    @Override
    public void runOpMode() {
        telemetry.update();

        activeGamepad1 = new Gamepad();

        previousGamepad1 = new Gamepad();

        manualMode = true;

        Mechanum = new MechanumDrive(hardwareMap);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            previousGamepad1.copy(activeGamepad1);
            activeGamepad1.copy(gamepad1);
            Intake.intake(activeGamepad1.a);

            if (activeGamepad1.back) {
                manualMode = !manualMode;
            }

            Mechanum.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if (activeGamepad1.options) {
                Mechanum.resetYaw();
            }

            if (manualMode) {

            } else {
            //Why is there an else? Idk
            }


            Mechanum.periodic(telemetry);
            telemetry.addData("Manual Mode: ", manualMode);
            telemetry.update();
        }
    }
}
