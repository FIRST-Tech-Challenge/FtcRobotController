package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.Map;

public class Drivetrain {
    Map<DriveMotors, DcMotor> motors = new HashMap<DriveMotors, DcMotor>();
    public void initMotors() {
        motors.put(DriveMotors.D1, hardwareMap.dcMotor.get("D1"));
        motors.put(DriveMotors.D2, hardwareMap.dcMotor.get("D2"));
        motors.put(DriveMotors.D3, hardwareMap.dcMotor.get("D3"));
        motors.put(DriveMotors.D4, hardwareMap.dcMotor.get("D4"));
    }

    public void drive() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double theta = Math.atan2(turn, x);
        double power = Math.hypot(x, turn);

        double sinA = Math.sin(theta - Math.PI / 4);
        double cosA = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sinA), Math.abs(cosA));

        motors.get(DriveMotors.D1).setPower(power * cosA/max + y);
        motors.get(DriveMotors.D2).setPower(power * sinA/max - y);
        motors.get(DriveMotors.D3).setPower(power * sinA/max + y);
        motors.get(DriveMotors.D4).setPower(power * cosA/max - y);

        if ((power + Math.abs(turn)) > 1) {
            motors.forEach((name, motor) -> {
                motor.setPower(motor.getPower() / (power + turn));
            });
        }
    }
}
