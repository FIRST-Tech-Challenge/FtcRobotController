package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoOpMode extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
            // Runs when the init button is pressed on driver hub

            waitForStart();

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() <= 1)) {
            // Runs for 1 second after start button is pressed

            }

            while (opModeIsActive()) {
            // Runs in a loop when the start button is pressed on the driver hub
            }
    }
}