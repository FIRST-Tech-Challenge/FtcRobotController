package org.firstinspires.ftc.teamcode.Opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hang;

@TeleOp
@Disabled

public class HangTest extends LinearOpMode {
    public void runOpMode(){
        Hang hang = new Hang(hardwareMap);
        waitForStart();
        while (opModeIsActive()&&!isStopRequested()){
            hang.move(gamepad1.left_stick_y);
        }
    }
}
