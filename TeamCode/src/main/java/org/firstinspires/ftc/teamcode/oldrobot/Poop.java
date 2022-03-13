package org.firstinspires.ftc.teamcode.oldrobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Poop extends LinearOpMode {
    @Override
    public void runOpMode() {
        final DcMotor[] motors = {
                hardwareMap.get(DcMotor.class, "fl"),
                hardwareMap.get(DcMotor.class, "fr"),
                hardwareMap.get(DcMotor.class, "bl"),
                hardwareMap.get(DcMotor.class, "br")
        };
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final GamepadEx toolGamepad = new GamepadEx(gamepad2);
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            final Double[] wheelPowers = (Double[]) MecanumKinematics.robotToWheelVelocities(new Pose2d(
                    moveGamepad.getLeftY(),
                    -moveGamepad.getLeftX(),
                    moveGamepad.getRightX()
            ), 1.0, 1.0).toArray();
            for (int i=0; i<4; i++) {
                motors[i].setPower(wheelPowers[i]);
            }
        }
    }
}
