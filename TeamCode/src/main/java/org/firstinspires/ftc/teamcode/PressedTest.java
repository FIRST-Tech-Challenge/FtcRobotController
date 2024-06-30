package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class PressedTest extends LinearOpMode {
    //hardwaremap here
    private Servo servo0;
    private ColorSensor colorSensor;
    private TouchSensor touchSensor;
    private DcMotor motor0;
    @Override
    public void runOpMode() {
        servo0 = hardwareMap.get(Servo.class, "servo0");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        motor0 = hardwareMap.get(DcMotor.class, "motor0");

        waitForStart();

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        boolean pressedLastIteration = false;
        double billy = 1.1;
        int bob = 1;


        while (opModeIsActive()) {

            boolean gamepad1A_pressed = gamepad1.a;
            boolean gamepad1B_pressed = gamepad1.b;

            if(touchSensor.isPressed() == true) {
                servo0.setPosition(0.7);
            }

            if (buttonHandler.isPressedOnceA(gamepad1A_pressed)) {
                servo0.setPosition(1.0);
                telemetry.addData("A:", gamepad1A_pressed);

            }
            if (buttonHandler.isPressedOnceB(gamepad1B_pressed)) {
                servo0.setPosition(0.4);
                telemetry.addData("B", gamepad1B_pressed);
            }


            motor0.setPower(gamepad1.left_stick_y);

            telemetry.addData("green", colorSensor.green());
            telemetry.addData("pressed", touchSensor.isPressed());
            telemetry.update();
        }

    }
}
