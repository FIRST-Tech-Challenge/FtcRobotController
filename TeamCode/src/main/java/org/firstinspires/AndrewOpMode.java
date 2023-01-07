package org.firstinspires;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.MecanumDrive;

import java.sql.Driver;

@TeleOp(name="AndrewOpMode", group="Linear Opmode")
public class AndrewOpMode extends LinearOpMode {

    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    InputHandler inputHandler;

    @Override
    public void runOpMode() {

        MecanumDrive driver = new MecanumDrive(
                hardwareMap.get(DcMotor.class, "motorBL"),
                hardwareMap.get(DcMotor.class, "motorFL"),
                hardwareMap.get(DcMotor.class, "motorBR"),
                hardwareMap.get(DcMotor.class, "motorFR")
        );

        inputHandler = new InputHandler(gamepad1);

        waitForStart();
        while (opModeIsActive()) {
            inputHandler.update();

            driver.move(
                inputHandler.getAnalogueInput(InputHandler.Analogue.LEFT_STICK_X),
                inputHandler.getAnalogueInput(InputHandler.Analogue.LEFT_STICK_Y),
                inputHandler.getAnalogueInput(InputHandler.Analogue.RIGHT_STICK_X)
            );

            if (inputHandler.justPressed(InputHandler.Digital.START)) {
                requestOpModeStop();
            }

        }
    }
}
