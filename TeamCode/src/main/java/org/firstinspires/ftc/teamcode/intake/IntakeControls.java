package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeControls {

    public DcMotor intake = null;
    private double _powerIntake = 0.0;
    private boolean _intake = false;
    private boolean _outtake = false;

    public void initialize(LinearOpMode op) {
        intake = op.hardwareMap.get(DcMotor.class, "Intake");
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setPower(0.0);
    }

    public void startControl() {

    }

    public void readController (Gamepad gamepad) {
        _intake = gamepad.right_bumper;
        _outtake = gamepad.left_bumper;

        if (_outtake == true) {
            _powerIntake = -1.0;
        } else if (_intake == true) {
            _powerIntake = 1.0;
        } else {
            _powerIntake = 0.0;
        }
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        this.readController(op.gamepad1);
        intake.setPower(_powerIntake);
    }

    public void addTelemetry (Telemetry telemetry) {

    }

    public void stop () {
        intake.setPower(0.0);
    }
}