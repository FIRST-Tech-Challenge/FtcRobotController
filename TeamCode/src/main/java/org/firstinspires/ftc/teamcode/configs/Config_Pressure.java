package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class Config_Pressure extends LinearOpMode {

    // declare variables here
    private TouchSensor ExtensionLimit;
    private TouchSensor LiftLimit;

    @Override
    public void runOpMode() {
        ExtensionLimit = hardwareMap.get(TouchSensor.class, "ExtensionLimit");
        LiftLimit = hardwareMap.get(TouchSensor.class, "LiftLimit");

        waitForStart();


        while (opModeIsActive()) {
            // do op mode things here
            if (ExtensionLimit.isPressed()) {
                telemetry.addLine("Extension Limit is pressed");
                //   telemetry.update();
                sleep(40);
            }

            if (!ExtensionLimit.isPressed()) {
                telemetry.addLine("Extension Limit is not pressed");
                //     telemetry.update();
                sleep(40);
            }
            if (LiftLimit.isPressed()) {
                telemetry.addLine("Lift Limit is pressed");
                //    telemetry.update();
                sleep(40);
            }
            if (!LiftLimit.isPressed()) {
                telemetry.addLine("Lift Limit is not pressed");
//  telemetry.update();
                sleep(40);
            }
            sleep(100);
            telemetry.update();

        }

    }
}