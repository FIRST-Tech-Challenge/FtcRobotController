package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "donga")

public class main extends LinearOpMode {

    movement movement = new movement();
    sensor sensor = new sensor();
    composetelemetry composetelemetry = new composetelemetry();


    @Override
    public void runOpMode() {
        //initialize motors
        movement.Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        movement.Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        movement.Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        movement.Motor4 = hardwareMap.get(DcMotor.class, "Motor4");

        //initialize sensor
        sensor.imu = hardwareMap.get(BNO055IMU.class, "imu");
        sensor.sensorIntitialize();
        sensor.imu.initialize(sensor.parameters);

        while (opModeIsActive()){
            gamepader();
        }
    }
    //gamepad input
    public void gamepader(){
        if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad1.right_stick_x)) {
            movement.verticalMovement(gamepad1.right_stick_y);
        } else if (Math.abs(gamepad1.right_stick_y) < Math.abs(gamepad1.right_stick_x)) {
            movement.horizontalMovement(gamepad1.right_stick_x);
        } else {
            movement.stop();
        }
    }
}
