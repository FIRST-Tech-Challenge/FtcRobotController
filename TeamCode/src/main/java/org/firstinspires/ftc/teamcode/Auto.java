package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GamepadStates;


@Autonomous(name = "Auto", group = "auto")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        boolean finished = false;
        boolean red = false;
        int delay = 0;
        int laps = 0;
        boolean left = false;

        drivetrain Drive = new drivetrain();

        Drive.init(this);

        GamepadStates newGamePad2 = new GamepadStates(gamepad1);

        telemetry.addLine("press b if red alliance");
        telemetry.addLine("press left for left side of the field");
        telemetry.addLine("press up to increase delay, or down to decrease");
        telemetry.addData("delay", delay);
        telemetry.update();

        if (newGamePad2.b.released) {
            red = true;
        }
        if (newGamePad2.dpad_left.released) {
            left = true;
        }
        if (newGamePad2.dpad_up.released) {
            if (delay > 15) {
                delay = 15;
            } else {
                delay += 1;
            }
        } else if (newGamePad2.dpad_down.released) {
            if (delay < 0) {
                delay = 0;
            } else {
                delay -= 1;
            }
        }


        waitForStart();

        sleep((long) (delay * 1000));

    }
}