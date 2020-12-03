package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Intake Test", group="Linear Opmode")

public class IntakeTest extends LinearOpMode {

    private Intake intake;


    @Override
    public void runOpMode() {


        intake = new Intake(
                hardwareMap.get(DcMotor.class, "topmotor"),
                hardwareMap.get(DcMotor.class, "botmotor")
        );
//

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad2.right_bumper) {
                intake.in();
            } else if (gamepad2.left_bumper) {
                intake.out();
            } else {
                intake.rest();
            }


            telemetry.addData("Status", "Run Time: ");
            telemetry.addData("Ly", gamepad1.left_stick_y);
            telemetry.addData("Lx", gamepad1.left_stick_x);
            telemetry.addData("Rx", gamepad1.right_stick_x);
            telemetry.update();

        }
    }
}
