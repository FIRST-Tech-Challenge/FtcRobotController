package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tools.*;
public class lifttestcode extends LinearOpMode {
    @Override
    public void runOpMode() {
        hardwareMap.get(Blinker.class, "Control Hub");
        hardwareMap.get(Blinker.class, "Expansion Hub 2");
        final GamepadEx controller = new GamepadEx(gamepad1);
        final Lift lift = new Lift(hardwareMap,controller);
        waitForStart();
        while (opModeIsActive()) {
            lift.update();
        }
    }
}
