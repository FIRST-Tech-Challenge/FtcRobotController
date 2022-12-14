package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.AccelerationDetector;
import org.firstinspires.ftc.teamcode.Functions.DataLogger.DataLogger22;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;

import java.io.IOException;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name="VLAD MARIAN IL IUBESTE PE HAMILTON <3", group="Test")
public class TestVladMarian extends OpMode {

    DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    Move move;
    Rotate rotate;
    DataLogger22 dataLogger22;
    RotationDetector rotationDetector;
    VoltageReader voltageReader;
    AccelerationDetector accelerationDetector;


    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        VoltageSensor VS =  this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        try {
            accelerationDetector = new AccelerationDetector(rotationDetector.ReturnGyro());
        } catch (IOException e) {
            e.printStackTrace();
        }
        dataLogger22 = new DataLogger22(rotationDetector,
                VS, move, accelerationDetector, "TestVladM");
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
    }

    @Override
    public void loop() {

        if(gamepad1.dpad_up) //merge in fata
        {
            move.MoveFull(1);
        }
        else if(gamepad1.dpad_down) //merge in spate
        {
            move.MoveFull(2);
        }
        else if(gamepad1.dpad_left) //merge stanga
        {
            rotate.RotateFull(1);

        }
        else if(gamepad1.dpad_right) // merge dreapta
        {
            rotate.RotateFull(2);
        }
        //miscarea stanga-dreapta:
        else if(gamepad1.right_bumper)
        {
            move.MoveFull(4);
        }
        else if(gamepad1.left_bumper)
        {
            move.MoveFull(3);
        }
        if(!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.right_bumper && !gamepad1.left_bumper
                && gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0 && gamepad1.right_stick_x==0 && !gamepad1.a && !gamepad1.b && gamepad1.right_stick_y==0)
        {
            move.MoveStop();
        }

        telemetry.addLine(dataLogger22.WriteData(getRuntime()));
        telemetry.update();


    }

}