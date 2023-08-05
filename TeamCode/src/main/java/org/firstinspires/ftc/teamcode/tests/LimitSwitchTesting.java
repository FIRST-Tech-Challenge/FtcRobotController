package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@Disabled
@TeleOp (name = "LimitSwitchTesting", group = "Tests")
public class LimitSwitchTesting extends LinearOpMode {
    DigitalChannel limitSwitch;

    @Override
    public void runOpMode() {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "rightLimit");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Switch State: ", limitSwitch.getState());
            telemetry.update();
        }
    }
}
