package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.AccelerationDetector;

import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.UltraSoundSensor;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;

@TeleOp(name="TestElisaUltra", group="TEST")
@Disabled
// Created by Elisa
public class TestElisaUltra extends OpMode {

    public Move move;
    public Rotate rotate;
    //public PositionCalculator positionCalculator;
    public RotationDetector rotationDetector;
    public VoltageReader voltageReader;
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    public UltraSoundSensor rangeSensor;

    AccelerationDetector accelerationDetector;
    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        MVVariables.MotorHolder motorHolder = new MVVariables.MotorHolder(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        move = new Move(motorHolder);
        rotate = new Rotate(motorHolder);
        VoltageSensor VS =  this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        try {
            accelerationDetector = new AccelerationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        } catch (Exception e) {
            e.printStackTrace();
        }
        try {
            rangeSensor = new UltraSoundSensor(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range"));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

//    @Override
//    //public void init_loop() {
//        telemetry.addLine(positionCalculator.CalibrateAccel());
//    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) //merge in fata
        {
            move.MoveFull(1);
            //telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));
            //positionCalculator.NewUpdate(getRuntime());

        }
        if(gamepad1.dpad_down) //merge in spate
        {
            move.MoveFull(2);
            //telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));
            //positionCalculator.NewUpdate(getRuntime());

        }
        if(gamepad1.dpad_left) //merge stanga
        {
            move.MoveFull(3);
            //telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));
            //positionCalculator.NewUpdate(getRuntime());

        }
        if(gamepad1.dpad_right) // merge dreapta
        {
            move.MoveFull(4);
            //telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));
            //positionCalculator.NewUpdate(getRuntime());
        }
        if(!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.a)
        {
            move.MoveStop();
        }

        //telemetry.addLine(positionCalculator.ReturnData());

        //telemetry.addLine(positionCalculator.NewUpdate(getRuntime()));
        //telemetry.addLine(positionCalculator.Update(getRuntime()));
        //rangeSensor.cmUltrasonic();
        telemetry.update();
    }

    void TelemetryData(){
        //telemetry.addLine(positionCalculator.ReturnData());
        telemetry.update();
    }
}
