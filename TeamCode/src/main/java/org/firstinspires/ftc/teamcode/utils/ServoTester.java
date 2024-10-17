package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "ServoTester")
public class ServoTester extends LinearOpMode {
    GamepadEvents controller1;
    CRServo CRservo;

    Servo servo;
    boolean savePower;
    double savePos1, savePos2;
    double savedPower;

    enum servoMode{
        DYNAMIC,
        PROGRAMMING,
        PROGRAMMED
    }
    servoMode programMode;
    @Override
    public void runOpMode() {


        try {
            CRservo = hardwareMap.get(CRServo.class, "CRservo");
            servo = hardwareMap.get(Servo.class, "servo");
            servo.setDirection(Servo.Direction.REVERSE);
        }
        catch (Exception e){
            telemetry.addLine("Ensure Continuous Rotation Servo is named CRservo");
            telemetry.addLine("Ensure standard servo is named servo");
        }
        controller1 = new GamepadEvents(gamepad1);

        savePower = false;
        programMode = servoMode.DYNAMIC;



        waitForStart();

        while(opModeIsActive()){
            switch (programMode){
                case DYNAMIC:
                    double pos = controller1.right_trigger.getTriggerValue();
                    servo.setPosition(pos);
                    if (controller1.b.onPress()){
                        programMode = servoMode.PROGRAMMED;
                    }
                    if (controller1.x.onPress()){
                        savePos1 = pos;
                    }
                    if (controller1.y.onPress()){
                        savePos2 = pos;
                    }
                    break;

                case PROGRAMMED:
                    if (controller1.b.onPress()){
                        programMode = servoMode.DYNAMIC;
                    }
                    if (controller1.x.onPress()){
                        servo.setPosition(savePos1);
                    }
                    if (controller1.y.onPress()){
                        servo.setPosition(savePos2);
                    }
                    break;
                default:
            }
            telemetry.addLine("X for Pos_1, Y for Pos_2, B to toggle to switch modes.");
            telemetry.addData("Position 1:", savePos1);
            telemetry.addData("Position 2:", savePos2);
            telemetry.addData("Mode: ", programMode);
            telemetry.addData("Current Position",servo.getPosition());

            double power = savePower ? savedPower : gamepad1.left_stick_y;
            savedPower = power;

            if (controller1.a.onPress()){
                savePower = !savePower;
            }
            CRservo.setPower(power);
            telemetry.addLine("Use left stick Y to power CRServo");
            telemetry.addData("Current Power: ",power);
            telemetry.addData("Locked Power Level: ", savePower);
            telemetry.update();
            controller1.update();
        }
    }
}
