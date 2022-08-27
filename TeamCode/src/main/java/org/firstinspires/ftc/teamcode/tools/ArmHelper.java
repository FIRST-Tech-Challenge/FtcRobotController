package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmHelper {
    private final Servo _arm, _grab;
    private final DcMotor _spinner;
    private final LinearOpMode _opMode;
    private final ElapsedTime _runtime;

    public ArmHelper(LinearOpMode opMode, String armName, String grabName, String spinnerName) {
        _opMode = opMode;
        _arm = _opMode.hardwareMap.get(Servo.class, armName);
        _grab = _opMode.hardwareMap.get(Servo.class, grabName);
        _spinner = _opMode.hardwareMap.get(DcMotor.class, spinnerName);

        _runtime = new ElapsedTime();
    }

    public void SetGrabPosition(double position) {
        _runtime.reset();
        _grab.setPosition(position);

        double curPos = _grab.getPosition();
        while (_runtime.time() < 2.0 && curPos != position) {
            curPos = _grab.getPosition();
            _opMode.telemetry.addData("Grab position", curPos);
            _opMode.telemetry.update();
        }
    }

    public void SetArmPosition(double position) {
        _runtime.reset();
        _arm.setPosition(position);

        double curPos = _grab.getPosition();
        while (_runtime.time() < 2.0 && curPos != position) {
            curPos = _grab.getPosition();
            _opMode.telemetry.addData("Arm position", curPos);
            _opMode.telemetry.update();
        }
    }

    public void Spin(double power, double seconds) {
        _runtime.reset();

        _spinner.setPower(power);

        double curTime = _runtime.time();
        while (_opMode.opModeIsActive() && curTime < seconds) {
            curTime = _runtime.time();
            _opMode.telemetry.addData("Spinning", power);
            _opMode.telemetry.update();
        }

        _spinner.setPower(0);
    }
}