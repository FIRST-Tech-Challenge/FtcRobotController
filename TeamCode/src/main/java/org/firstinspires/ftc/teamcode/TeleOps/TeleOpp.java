package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp
public class TeleOpp extends LinearOpMode {

    private Drivetrain drive = new Drivetrain();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double strafe = -gamepad1.left_stick_y;
            double power = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drive.move(power, strafe, turn);

            telemetry.addData("Power", power);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.update();
        }
    }
}
