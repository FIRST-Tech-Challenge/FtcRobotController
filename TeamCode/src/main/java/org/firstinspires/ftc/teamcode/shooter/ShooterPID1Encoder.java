package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterPID1Encoder {

    // Declare OpMode members.
    public ElapsedTime _runtime = new ElapsedTime();
    public DcMotorEx _primaryMotor = null;
    public DcMotorEx _secondaryMotor = null;

    public double _lastTime = _runtime.milliseconds();
    public double _lastTicks = 0.0;
    public double _lastError = 0.0;
    public double _error = 0.0; // ticks / second
    public double _Integral = 0.0; // Sum of error (ticks / second)
    public double _Derivative = 0.0; // Delta error / delta time
    public double _targetRPM = 0.0; // Setpoint in RPM
    public double _targetTicksPerSecond = 0.0; // Setpoint in ticks / second
    public double _measuredTicksPerSecond = 0.0;
    public double _measuredRPM = 0.0;
    public double _currentPower = 0.0;
    public boolean _yButtonPressed = false;
    public boolean _aButtonPressed = false;

    public static final double MOTOR_TICKS_PER_REVOLUTION = 103.6;
    public static final double SECONDS_PER_MINUTE = 60;
    public static final double EXTERNAL_GEAR_RATIO = 3.0 / 1.0;
    public static final double MIN_RPM = 0.0;
    public static final double MAX_RPM = (1780 * EXTERNAL_GEAR_RATIO);
    public static final double RPM_LIMIT = MAX_RPM * 0.9;
    public final double RPM_INCREMENT = 50;

    public static final double RPM_TO_TICKS_PER_SECOND (double rpm) {
        return (rpm * MOTOR_TICKS_PER_REVOLUTION) / (SECONDS_PER_MINUTE * EXTERNAL_GEAR_RATIO);
    }

    public static final double TICKS_PER_SECOND_TO_RPM (double ticksPerSecond) {
        return (ticksPerSecond * SECONDS_PER_MINUTE * EXTERNAL_GEAR_RATIO) / (MOTOR_TICKS_PER_REVOLUTION);
    }

    static final double MAX_TICKS_PER_SECOND = RPM_TO_TICKS_PER_SECOND(MAX_RPM);
    public double _P = 1.0;
    public double _I = 1.0;
    public double _D = 0.0;

    public boolean initialize(LinearOpMode op){
        _primaryMotor = (DcMotorEx)op.hardwareMap.get(DcMotor.class, "ShootRgt");
        if (_primaryMotor == null) {
            op.telemetry.addData("Primary Motor", "Null");
            return false;
        }
        _primaryMotor.setPower(0.0);
        _primaryMotor.setDirection(DcMotor.Direction.FORWARD);
        _primaryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        _primaryMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _secondaryMotor = (DcMotorEx)op.hardwareMap.get(DcMotor.class, "ShootLft");
        if (_secondaryMotor == null) {
            op.telemetry.addData("Secondary Motor", "Null");
            return false;
        }
        _secondaryMotor.setPower(0.0);
        _secondaryMotor.setDirection(DcMotor.Direction.FORWARD);
        _secondaryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        _secondaryMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        op.idle();
        _primaryMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return true;
    }

    public void startControl(){
        _lastTime = _runtime.seconds();
        _lastTicks = _primaryMotor.getCurrentPosition();
        _lastError = 0.0;
        _error = 0.0;
        _Integral = 0.0;
        _Derivative = 0.0;
        _measuredTicksPerSecond = 0.0;
        _measuredRPM = 0.0;
    }

    public void setByRPM (double targetRPM) {
        _targetRPM = Range.clip(targetRPM, MIN_RPM, RPM_LIMIT);
        _targetTicksPerSecond = RPM_TO_TICKS_PER_SECOND(_targetRPM);
    }

    public void whileOpModeIsActive () {
        double currentTime = _runtime.seconds();
        double deltaTime = currentTime - _lastTime;
        if (deltaTime < 0.1) {
            return;
        }
        double currentTicks = _primaryMotor.getCurrentPosition();
        double deltaTicks = currentTicks - _lastTicks;
        _measuredTicksPerSecond = deltaTicks / deltaTime;
        _measuredRPM = TICKS_PER_SECOND_TO_RPM(_measuredTicksPerSecond);

        _error = _targetTicksPerSecond - _measuredTicksPerSecond;
        double deltaError = _error - _lastError;
        _Derivative = deltaError / deltaTime;
        _Integral += (_error * deltaTime);

        _currentPower = (_P * _error + _I * _Integral + _D * _Derivative) / MAX_TICKS_PER_SECOND;
        _currentPower = Range.clip(_currentPower, 0.0, 1.0);
        _primaryMotor.setPower(_currentPower);
        _secondaryMotor.setPower(_currentPower);

        _lastTime = currentTime;
        _lastTicks = currentTicks;
        _lastError = _error;
    }

    public void stop () {
        _primaryMotor.setPower(0.0);
        _secondaryMotor.setPower(0.0);
    }

    public void readController (Gamepad gamepad) {
        double newRPM = _targetRPM;

        if (!_yButtonPressed && gamepad.y) {
            newRPM = Range.clip(newRPM + RPM_INCREMENT, MIN_RPM, RPM_LIMIT);
            this.setByRPM(newRPM);
        } else if (!_aButtonPressed && gamepad.a) {
            newRPM = Range.clip(newRPM - RPM_INCREMENT, MIN_RPM, RPM_LIMIT);
            this.setByRPM(newRPM);
        }
        _yButtonPressed = gamepad.y;
        _aButtonPressed = gamepad.a;
    }

    public void addTelemetry (Telemetry telemetry) {
        telemetry.addData("Target RPM", "%.03f rpm", _targetRPM);
        telemetry.addData("Measured RPM", "%.03f rpm", _measuredRPM);
        telemetry.addData("Target TPS", "%.03f tps", _targetTicksPerSecond);
        telemetry.addData("Measured TPS", "%.03f tps", _measuredTicksPerSecond);
        telemetry.addData("Power", "%.03f %%", _currentPower);
    }

}