package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class DrivetrainHardware {
    
    public DcMotor FL; //Motor 0 - LM0
    public DcMotor BL; //Motor 1 - LM1
    public DcMotor BR; //Motor 2 - RM2
    public DcMotor FR; //Motor 3 - RM3
    // public ;
    public DcMotor Intake; //Motor 0
    public DcMotorEx FlyWheel1; //Motor 1
    public DcMotorEx FlyWheel2; //Motor 2
    public DcMotor Arm; //Motor 3

    public Servo ringHopper, claw; //0, 1
    public CRServo intake1; // intake2; //2, 3

    public BNO055IMU imu; //bus 0


    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".

    /*
    public DistanceSensor distanceFrontLeft;
    public DistanceSensor distanceFrontRight;
    public DistanceSensor distanceBackLeft;
    public DistanceSensor distanceBackRight;
    */

    public RevBlinkinLedDriver blink; //3

    //public VoltageSensor voltageSensor;

    HardwareMap hardwareMap;
    
    public DrivetrainHardware()
    {
        FL = null;
        BL = null;
        BR = null;
        FR = null;
        FlyWheel1 = null;
        FlyWheel2 = null;
        Intake = null;
        intake1 = null;
        //intake2 = null;
        hardwareMap = null;
        ringHopper = null;
        Arm = null;
        claw = null;
        blink = null;

        /*
        distanceFrontRight = null;
        distanceFrontLeft = null;
        distanceBackRight = null;
        distanceBackLeft = null;
        */

    }
    
    public void init(HardwareMap h)
    {
        hardwareMap = h;

        BR = hardwareMap.get(DcMotor.class, "M0");
        FR = hardwareMap.get(DcMotor.class, "M1");
        FL = hardwareMap.get(DcMotor.class, "M2");
        BL = hardwareMap.get(DcMotor.class, "M3");
        FlyWheel1 = hardwareMap.get(DcMotorEx.class, "FW1");
        FlyWheel2 = hardwareMap.get(DcMotorEx.class, "FW2");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Arm = hardwareMap.get(DcMotor.class, "Arm");

        ringHopper = hardwareMap.get(Servo.class, "hopper"); //0
        claw = hardwareMap.get(Servo.class, "claw"); //1
        //intake1 = hardwareMap.get(CRServo.class, "intake1");
        //intake2 = hardwareMap.get(CRServo.class, "intake2");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /*
        distanceFrontLeft = hardwareMap.get(DistanceSensor.class, "bus0");
        distanceFrontRight = hardwareMap.get(DistanceSensor.class, "bus1");
        distanceBackLeft = hardwareMap.get(DistanceSensor.class, "bus2");
        distanceBackRight = hardwareMap.get(DistanceSensor.class, "bus3");
        */

        blink = hardwareMap.get(RevBlinkinLedDriver.class, "blink");
        blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);


        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FlyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FlyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        freeze();

        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        FlyWheel1.setDirection(DcMotor.Direction.REVERSE);
        FlyWheel2.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();


    }


    public void freeze()
    {
        FL.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FR.setPower(0);
        FlyWheel1.setPower(0);
        FlyWheel2.setPower(0);
        Intake.setPower(0);
        Arm.setPower(0);
    }

    public double getVoltage()
    {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void resetEncoders()
    {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FlyWheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FlyWheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FlyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FlyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }


    public double getEncoderAvg()
    { return ((Math.abs(FR.getCurrentPosition()) + Math.abs(BR.getCurrentPosition())) / 2.0); }

    /*
    public double getEncoderAvg() {
        double output = 0;
        int encoderCount = 0;
        boolean[] encoderIsNotPluggedIn = new boolean[4];

        if (Math.abs(BR.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(BR.getCurrentPosition());
        } else encoderIsNotPluggedIn[0] = true;

        if (Math.abs(FR.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(FR.getCurrentPosition());
        } else encoderIsNotPluggedIn[1] = true;

        if (Math.abs(FL.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(FL.getCurrentPosition());
        } else encoderIsNotPluggedIn[2] = true;

        if (Math.abs(BL.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(BL.getCurrentPosition());
        } else encoderIsNotPluggedIn[3] = true;

        if (encoderCount == 0)
            return 0;
        else
            return output/encoderCount;
    }*/
}
