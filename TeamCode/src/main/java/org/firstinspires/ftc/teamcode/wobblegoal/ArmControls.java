package org.firstinspires.ftc.teamcode.wobblegoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmControls {

    public DcMotor arm = null;
    private double _powerArm = 0.0;
    private double _encoderArm = 0.0;
    private double _lowLimit = 50.0;
    private double _highLimit = 9000.0;
    private double _y = 0.0;

    public void initialize(LinearOpMode op) {
        arm = op.hardwareMap.get(DcMotor.class, "WobbleArm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(0.0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void startControl() {

    }

    public void readController (Gamepad gamepad) {
        _y = -gamepad.left_stick_y;
    }

    public void whileOpModeIsActive (LinearOpMode op) {
        this.readController(op.gamepad2);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _encoderArm = arm.getCurrentPosition();

        arm.setPower(_powerArm);

        if ((_encoderArm <= _lowLimit) && (_y < 0)) {
            _powerArm = 0.0;
        } else if ((_encoderArm >= _highLimit) && (_y > 0)) {
            _powerArm = 0.0;
        } else {
            _powerArm = 1.0 * _y;
        }

    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("Wobble Goal Arm Encoder", "%f", _encoderArm);
    }

    public void stop () {
        arm.setPower(0.0);
    }
}