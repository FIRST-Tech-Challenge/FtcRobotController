package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.helpers.Constants;

import java.util.EnumMap;
import java.util.Map;

@TeleOp(name = "TestChassis", group = "TeleOp")
public class TestJustDriveOpMode extends OpMode{
    protected DriveSystem driveSystem;

        @Override
        public void init() {

            Map<DriveSystem.MotorNames, DcMotor> motors = new EnumMap<>(DriveSystem.MotorNames.class);
            motors.put(DriveSystem.MotorNames.FRONTRIGHT, hardwareMap.get(DcMotor.class, Constants.MOTOR_FRONT_RIGHT));
            motors.put(DriveSystem.MotorNames.FRONTLEFT, hardwareMap.get(DcMotor.class, Constants.MOTOR_FRONT_LEFT));
            motors.put(DriveSystem.MotorNames.BACKRIGHT, hardwareMap.get(DcMotor.class, Constants.MOTOR_BACK_RIGHT));
            motors.put(DriveSystem.MotorNames.BACKLEFT, hardwareMap.get(DcMotor.class, Constants.MOTOR_BACK_LEFT));

            driveSystem = new DriveSystem(motors, hardwareMap.get(BNO055IMU.class, Constants.IMU));
        }

        @Override
        public void start() {
            super.start();
        }

    @Override
    public void loop() {
        float rx = (float) Math.pow(gamepad1.right_stick_x, 3);
        float lx = (float) Math.pow(gamepad1.left_stick_x, 3);
        float ly = (float) Math.pow(gamepad1.left_stick_y, 3);

        driveSystem.drive(rx, -lx, ly);
    }


}