package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.HashMap;
import java.util.Map;
@TeleOp(name="Normal")
public class Teleop extends OpMode {

    Map<DriveMotors, DcMotor> motors = new HashMap<DriveMotors, DcMotor>();
    IMU gyro = null;
    public void initMotors() {
        motors.put(DriveMotors.D1, hardwareMap.dcMotor.get("D1"));
        motors.put(DriveMotors.D2, hardwareMap.dcMotor.get("D2"));
        motors.put(DriveMotors.D3, hardwareMap.dcMotor.get("D3"));
        motors.put(DriveMotors.D4, hardwareMap.dcMotor.get("D4"));
    }

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        initMotors();
//        gyro.initialize()
//        packet.put("x", 0.0);
    }

    public void loop() {
        motors.get(DriveMotors.D1).setPower(gamepad1.left_stick_x);

    }

    public void drive() {
//        double sinA = Math.sin()
        motors.forEach((name, dcMotor) -> {
            switch (name) {
                case D1:

            }
        });
    }
}