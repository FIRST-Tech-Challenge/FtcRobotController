package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;


@TeleOp
public class MainTeleOp extends LinearOpMode {

    private DcMotorEx motor_fr;
    private DcMotorEx motor_fl;
    private DcMotorEx motor_br;
    private DcMotorEx motor_bl;
    private IMU imu;
    private GamepadEx gp1, gp2;
    Bot bot;

   /* Slides slides;
    Noodles noodles;
    Transfer transfer;

    */

    private double driveSpeed=1;

    @Override
    public void runOpMode() throws InterruptedException {
        motor_fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        motor_fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        motor_br = hardwareMap.get(DcMotorEx.class, "backRight");
        motor_bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        imu = hardwareMap.get(IMU.class, "IMU?");
        gp2 = new GamepadEx(gamepad2);
        gp1 = new GamepadEx(gamepad1);


        telemetry.addData("teleOp is ", "initialized");

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("teleOp is ", "running");
            telemetry.update();
            gp1.readButtons();
            gp2.readButtons();


        }


    }
    private void drive() {
        if (gp1.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            bot.resetIMU();
        }
        driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);

        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX(), 0);
        bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed / 1.7
        );
    }
}
