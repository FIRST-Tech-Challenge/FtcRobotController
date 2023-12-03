package org.firstinspires.ftc.team6220_CENTERSTAGE.teleOpClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Utilities;

@Disabled
@TeleOp(name =
        "IntakeTestPoggersGaming")
public class IntakeTest extends MainTeleOp {

    MecanumDrive drive;

    double intakeServoPosition = 0.5;



    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            intakeServoPosition = Utilities.clamp(intakeServoPosition + (gamepad2.right_stick_y * 0.001), 0.0, 1.0);
            drive.intakeServo.setPosition(intakeServoPosition);

            telemetry.addData("Servo Position: ", intakeServoPosition);
            telemetry.update();
        }
    }
}
