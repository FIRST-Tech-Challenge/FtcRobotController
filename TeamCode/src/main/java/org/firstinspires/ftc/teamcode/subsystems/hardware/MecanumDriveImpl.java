package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class MecanumDriveImpl implements MecanumDrive {

    protected DcMotor [] _motors;
    protected Telemetry _telemetry;
    protected int [] _FREE_WHEELS; // no encoder wheels (RIGHT, LEFT)
    protected int [] _ENCODER_WHEELS; // encoder wheels (RIGHT, LEFT)
    protected int [] _REVERSED_WHEELS; // reversed motors list


    protected Telemetry.Item T_FrontRightSpeed, T_FrontLeftSpeed, T_RearRightSpeed,
            T_RearLeftSpeed, T_FrontRightPosition, T_FrontLeftPosition, T_RearRightPosition, T_RearLeftPosition;


    public MecanumDriveImpl(MecanumDriveParameters parameters) {
        init(parameters.motors,parameters.telemetry,parameters.FREE_WHEELS,parameters.ENCODER_WHEELS,parameters.REVERSED_WHEELS);
    }

    public MecanumDriveImpl(DcMotor[] motors, Telemetry telemetry,
                int[] FREE_WHEELS, int[] ENCODER_WHEELS, int[] REVERSED_WHEELS) {
        init(motors,telemetry,FREE_WHEELS,ENCODER_WHEELS,REVERSED_WHEELS);
    }



    protected void init(DcMotor[] _motors, Telemetry _telemetry,
                            int[] _FREE_WHEELS, int[] _ENCODER_WHEELS, int[] _REVERSED_WHEELS) {
        this._motors = _motors;
        this._telemetry = _telemetry;
        this._FREE_WHEELS = _FREE_WHEELS;
        this._ENCODER_WHEELS = _ENCODER_WHEELS;
        this._REVERSED_WHEELS = _REVERSED_WHEELS;
        setRunMode(_FREE_WHEELS, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(_REVERSED_WHEELS, DcMotorSimple.Direction.REVERSE);
        setZeroPowerBehavior(new int[]{0,1,2,3},DcMotor.ZeroPowerBehavior.BRAKE);


        // set up telemetry objects:
        T_FrontRightSpeed = _telemetry.addData("FRS","0");
        T_FrontLeftSpeed = _telemetry.addData("FLS","");
        T_RearRightSpeed = _telemetry.addData("RRS","");
        T_RearLeftSpeed = _telemetry.addData("RLS","");
        T_FrontRightPosition = _telemetry.addData("FRP","");
        T_FrontLeftPosition = _telemetry.addData("FLP","");
        T_RearRightPosition = _telemetry.addData("RRP","");
        T_RearLeftPosition = _telemetry.addData("RLP","");

    }


    @Override
    public void moveRect(double forward, double lateral, double rotate) {
        //translate into polar and move accordingly.
        movePolar(Math.hypot(forward,lateral),
                Math.atan2(-forward,lateral),
                rotate);
    }

    @Override
    public void movePolar(double power, double angle, double rotate) {
        angle -= Constants.PI_OVER4;
        rotate *= Constants.ROTATION_RATE;
        double sine  = Math.sin(angle);
        double cosine = Math.cos(angle);
        double scale = ( (power + Math.abs(rotate)) > 1 ) ? Constants.SPEED_FACTOR /(power + rotate) : Constants.SPEED_FACTOR /Math.sqrt(2) ;

        double [] wheelSpeeds = {
                scale * (power * sine - rotate),   // Front Right
                scale * (power * cosine - rotate), // Rear Right
                scale * (power * sine + rotate),   // Rear Left
                scale * (power * cosine + rotate)  // Front Left
        };
        setMotorSpeeds(wheelSpeeds);
        outputTelemetry(MecanumDriveTelemetryTypes.WHEEL_SPEEDS);
        outputTelemetry(MecanumDriveTelemetryTypes.WHEEL_POSITIONS);
    }

    @Override
    public void outputTelemetry(MecanumDriveTelemetryTypes type) {
        switch (type) {
            case WHEEL_SPEEDS:
                double speeds []  = readSpeeds();
                T_RearLeftSpeed.setValue(speeds[WheelPositions.RearLeft.position]);
                T_RearRightSpeed.setValue(speeds[WheelPositions.RearRight.position]);
                T_FrontLeftSpeed.setValue(speeds[WheelPositions.FrontLeft.position]);
                T_FrontRightSpeed.setValue(speeds[WheelPositions.FrontRight.position]);
                break;
            case WHEEL_POSITIONS:
                int encoders []  = readEncoders();
                T_RearLeftPosition.setValue(encoders[WheelPositions.RearLeft.position]);
                T_RearRightPosition.setValue(encoders[WheelPositions.RearRight.position]);
                T_FrontLeftPosition.setValue(encoders[WheelPositions.FrontLeft.position]);
                T_FrontRightPosition.setValue(encoders[WheelPositions.FrontRight.position]);
                break;
        }

    }

    public int [] readEncoders() {  // return the encoder values if there are encoder motors.
        if (_ENCODER_WHEELS != null && _ENCODER_WHEELS.length > 0 ) {
            int [] encoders = new int[_ENCODER_WHEELS.length];
            for (int i=0; i<_ENCODER_WHEELS.length; i++) {
                encoders[i] = _motors[_ENCODER_WHEELS[i]].getCurrentPosition();
            }
            return encoders;
        }
        return null;
    }

    public double[] readSpeeds() {
        if (_motors != null) {
            double [] speeds = new double[_motors.length];
            for (int i=0; i< _motors.length; i++) {
                speeds[i] = _motors[i].getPower();
            }
            return speeds;
        }
        return null;
    }


    protected void setDirection(int [] wheels, DcMotorSimple.Direction dir) {
        if (wheels != null && wheels.length != 0)
            for (int wheel : wheels)
                _motors[wheel].setDirection(dir);

    }

    protected void setMotorSpeeds(double [] speeds) {
        for (int i = 0; i < _motors.length; i++) _motors[i].setPower(Range.clip(speeds[i], -1, 1));
    }

    protected void setRunMode(int [] wheels, DcMotor.RunMode mode) {
        if (wheels != null && wheels.length != 0)
            for (int wheel : wheels)   // for (int wheel =0; wheel< wheels.length; wheel++)
                _motors[wheel].setMode(mode);
    }

    protected void setZeroPowerBehavior( int [] wheels, DcMotor.ZeroPowerBehavior behavior) {
        if (wheels != null && wheels.length != 0)
            for (int wheel : wheels)
                _motors[wheel].setZeroPowerBehavior(behavior);
    }


}
