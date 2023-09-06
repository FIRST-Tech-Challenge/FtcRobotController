package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SubSystem;
import org.firstinspires.ftc.teamcode.threads.RobotThread;

public class LiftClaw extends RobotThread implements SubSystem {

    public final static double SERVO_POS_OPEN = 0;
    public final static double SERVO_POS_CLOSE = .45;
    public final static int BOTTOM_POS = 0;   //1120 ticks per revolution
    public final static int LOW_POS = 2300;
    public final static int MEDIUM_POS = 3600;
    public final static int HIGH_POS = 5475;

    public final static int STACK_TOP_PICKUP=800;
    public final static int STACK_INCREMENT=200;

    // static points
    final static int CLAW_A = 0;
    final static int CLAW_B = 1;


    // hardware objects
    DcMotor _DriveMotor;
    Servo[] _clawServos;
    //Servo _pipe_guide;
    TouchSensor _bottomStopSensor;
    DistanceSensorDevice _releaseSensor;
    Telemetry _telemetry;
    Lights _light;

    //telemetry items
    Telemetry.Item _T_pos,_B_stop,_C_STAT,P_GUIDE;
    long _clawOpenTime=0;


    //34.75" from bottom to top
    /*
    1540 top of 5 stack


   cone stack heights
     1st 0
     2nd 200
     3rd 400
     4th 600
     5th 800
     */

    public LiftClaw(DcMotor Motor, Servo [] servos, TouchSensor stop,
                    DistanceSensorDevice release, Telemetry telemetry, Lights light) {
        _light = light;
        _telemetry = telemetry;
        _DriveMotor = Motor;
        _clawServos = new Servo[2];
        for (int i = 0; i < servos.length; i++) {
            servos[i].setDirection(Servo.Direction.FORWARD);
            _clawServos[i] = servos[i];
        }

        _C_STAT = telemetry.addData("Claw", "open");
        clawOpen();
        clawClose();
        _bottomStopSensor = stop;
        _releaseSensor = release;
        _T_pos = telemetry.addData("Lift", 0);
        _B_stop = telemetry.addData("BSTOP", _bottomStopSensor.isPressed());
        _DriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        P_GUIDE = _telemetry.addData("PIPE GUIDE", "unknown");
    }

    public void calibrateLift() {

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!_bottomStopSensor.isPressed()) {
            _DriveMotor.setPower(-0.5);
        }
        _DriveMotor.setPower(0);
        resetEncoder();
    }

    public boolean checkOptical() {
        return (_releaseSensor.checkDistanceMM(30));
    }


    public void clawClose() {
        if (System.currentTimeMillis() > _clawOpenTime+1500) {
            _clawServos[CLAW_A].setPosition(1 - SERVO_POS_CLOSE);
            _clawServos[CLAW_B].setPosition(SERVO_POS_CLOSE);
            _C_STAT.setValue("closed");
        }
    }

    public void clawOpen() {
        _clawServos[CLAW_A].setPosition(SERVO_POS_OPEN);
        _clawServos[CLAW_B].setPosition(1- SERVO_POS_OPEN);
        _C_STAT.setValue("open");
    }

    public int getEncoder() {
        return _DriveMotor.getCurrentPosition();
    }

    public void moveLift(double speed) { // move with the joystick
        if (!_bottomStopSensor.isPressed()) {
            _DriveMotor.setPower(Range.clip(-speed, -1, 1));
        } else {
            resetEncoder();
            if (-speed > 0) {
                _DriveMotor.setPower(Range.clip(-speed, -1, 1));
            } else {
                _DriveMotor.setPower(0);
            }
        }
        _T_pos.setValue(getEncoder());
    }

    public void placeCone() {
        _DriveMotor.setPower(-1);
        while (true) {
            if (checkOptical()) break;
            if (_DriveMotor.getCurrentPosition() < 100  || _DriveMotor.getCurrentPosition() > 5300) break;

        }
        clawOpen();
        _DriveMotor.setPower(0);
    }

    public void placeCone(long pos) {
        _DriveMotor.setPower(-1);
        while (true) {
            if (checkOptical()) break;
            if (_DriveMotor.getCurrentPosition() < pos  || _DriveMotor.getCurrentPosition() > 5300) {
                clawOpen();
                break;
            }

        }
        clawOpen();
        _DriveMotor.setPower(0);
    }




    public void resetEncoder() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _T_pos.setValue(getEncoder());
    }

    public void runToPos(int target) {
        runToPos(target,1.0);
    }

    public void runToPos(int target, double speed) {
        int current, last;
        _DriveMotor.setTargetPosition(target);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(_DriveMotor, -(Math.signum(getEncoder()-target)*speed));
        last = -100;
        ElapsedTime et = new ElapsedTime();
        et.reset();
        long start_time = System.currentTimeMillis();
        _stop_triggered = false;
        while (true) {
            current = _DriveMotor.getCurrentPosition();
            if ((System.currentTimeMillis() > start_time + 100) && (current == last)) {
                _telemetry.log().add("current("+current+") == last("+last+")");
                break;
            }
            last = current;
            if (!_DriveMotor.isBusy()) {
                _telemetry.log().add("motor is not busy");
                break;
            }
            if (_stop_triggered) {
                break;
            }
        }
        setPower(_DriveMotor,0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected boolean _stop_triggered=false;
    public void triggerStop() {
        _stop_triggered = true;
    }

    public void setClawOpenTime(long _clawOpenTime) {
        this._clawOpenTime = _clawOpenTime;
    }

    public void setMode(DcMotor.RunMode mode) {
        _DriveMotor.setMode(mode);
    }

    public void setPower(DcMotor motor, double power) {
        motor.setPower(power);
    }


}
