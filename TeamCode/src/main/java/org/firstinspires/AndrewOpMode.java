package org.firstinspires;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.MecanumDrive;

import java.sql.Driver;

@TeleOp(name="Andrew Mecanum Drive", group="Linear Opmode")
public class AndrewOpMode extends LinearOpMode {

    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;


    boolean storage;
    boolean slowMode;
    double movement_mul;

    @Override
    public void runOpMode() {

        MecanumDrive driver = new MecanumDrive(
                hardwareMap.get(DcMotor.class, "motorBL"),
                hardwareMap.get(DcMotor.class, "motorFL"),
                hardwareMap.get(DcMotor.class, "motorBR"),
                hardwareMap.get(DcMotor.class, "motorFR")
        );


        movement_mul = 1.0;
        storage = false;
        slowMode = false;

        waitForStart();
        while (opModeIsActive()) {

            if (!storage && gamepad1.a) {
                slowMode = !slowMode;
            }

            if (slowMode) {
                movement_mul = 0.3;
            } else {
                movement_mul = 1.0;
            }

            storage = gamepad1.a;
            driver.move(
                movement_mul * gamepad1.left_stick_x,
                movement_mul * gamepad1.left_stick_y,
                movement_mul * gamepad1.right_stick_x
            );

            if (gamepad1.start) {break;}

            telemetry.addData("Slow Mode", slowMode);

        }
    }
}
