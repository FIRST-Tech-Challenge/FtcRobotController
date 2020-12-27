package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Teleop", group = "Linear Opmode")
@Disabled
public class IntakeTest extends LinearOpMode {
    public DcMotor intake = null;

    private double _powerIntake = 0.0;
    private boolean _intake = false;
    private boolean _outtake = false;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPower(0.0);

        waitForStart();

        while(opModeIsActive()) {
            _outtake = gamepad1.left_bumper;
            _intake = gamepad1.right_bumper;

            if (_outtake == true) {
                _powerIntake = -1.0;
            } else if (_intake == true) {
                _powerIntake = 1.0;
            } else {
                _powerIntake = 0.0;
            }

            intake.setPower(_powerIntake);

            telemetry.update();
            idle();
        }
        intake.setPower(0.0);
    }
}
