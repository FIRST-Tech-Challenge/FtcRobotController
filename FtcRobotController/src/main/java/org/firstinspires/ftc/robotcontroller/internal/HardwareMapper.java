package org.firstinspires.ftc.robotcontroller.internal;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

public class HardwareMapper {
    private robotBase base;

    public HardwareMapper(robotBase Robotbase){
        this.base = Robotbase;
    }

    public DcMotor mapMotor(final String NAME, final DcMotorSimple.Direction Direction, final DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        DcMotor temporaryMotor;
        temporaryMotor = base.getHardwaremap().dcMotor.get(NAME);
        temporaryMotor.setDirection(Direction);
        temporaryMotor.setZeroPowerBehavior(zeroPowerBehavior);

        return temporaryMotor;
    }
    public DcMotor mapMotor(final String NAME, final DcMotorSimple.Direction DIRECTION){
        DcMotor tempMotor;
        tempMotor = base.getHardwaremap().get(DcMotor.class, NAME);
        tempMotor.setDirection(DIRECTION);

        return tempMotor;
    }
    public DcMotor mapMotor(final String NAME){
        DcMotor temporaryMotor;
        temporaryMotor = base.getHardwaremap().get(DcMotor.class, NAME);

        return temporaryMotor;
    }
    public Servo mapServo(final String NAME){
        Servo temporaryServo;
        temporaryServo =base.getHardwaremap().servo.get(NAME);

        return temporaryServo;
    }

}
