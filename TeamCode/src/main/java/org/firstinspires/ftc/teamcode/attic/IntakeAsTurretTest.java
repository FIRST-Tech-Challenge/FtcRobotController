package org.firstinspires.ftc.teamcode.attic;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.OrangeBotBaseOpMode;

@Disabled
@TeleOp(name = "Intake Encoder Test", group = "Test")
public class IntakeAsTurretTest extends OrangeBotBaseOpMode {

    boolean intakeOn = false;
    boolean reverse = false;
    double intakePower = 0.2;
    static final double INTAKE_INCREMENT = 0.05;

    @Override
    public void runOpMode() { // when driver hits INIT
        config();

        waitForStart();

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {

            if (gamepad1.yWasPressed()) {
                intakeOn = !intakeOn;
            }
            if (gamepad1.xWasPressed()) {
                reverse = !reverse;
            }

            if (gamepad1.dpadUpWasPressed()) {
                intakePower += INTAKE_INCREMENT;
                if (intakePower > 1.0) {
                    intakePower = 1.0;
                }
            }
            if (gamepad1.dpadDownWasPressed()) {
                intakePower -= INTAKE_INCREMENT;
                if (intakePower < 0.0) {
                    intakePower = 0.0;
                }
            }

            double thePower = reverse ? -intakePower : intakePower;
            intake.setPower(intakeOn ? thePower : 0.0);

            telemetry.addData("Current position", intake.getCurrentPosition());
            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }

    }


}