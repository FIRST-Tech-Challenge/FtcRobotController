package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake + Outtake", group = "TeleOp")
public class TeleOp_InAndOuttake_Hinaa extends LinearOpMode {

    private DcMotor intake;
    private DcMotor outtakeleft;
    private DcMotor outtakeright;

    @Override
    public void runOpMode() {

        // Map motors to config names in the RC configuration
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeleft = hardwareMap.get(DcMotor.class, "outtakeleft");
        outtakeright = hardwareMap.get(DcMotor.class, "outtakeright");

        // Set directions (one of the outtakes should spin opposite)
        outtakeleft.setDirection(DcMotor.Direction.FORWARD);
        outtakeright.setDirection(DcMotor.Direction.REVERSE);

        // Stop motors initially
        intake.setPower(0);
        outtakeleft.setPower(0);
        outtakeright.setPower(0);

        telemetry.addLine("Ready — press PLAY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Intake control — use right trigger for intake in, left trigger for reverse
            double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
            intake.setPower(intakePower);

            // Outtake control — press 'A' to run both outtakes, 'B' to reverse
            double outtakePower = 0;
            if (gamepad2.a) {
                outtakePower = 1.0; // full forward
            } else if (gamepad2.b) {
                outtakePower = -1.0; // reverse
            }

            outtakeleft.setPower(outtakePower);
            outtakeright.setPower(outtakePower);

            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Outtake Power", outtakePower);
            telemetry.update();
        }
    }
}
