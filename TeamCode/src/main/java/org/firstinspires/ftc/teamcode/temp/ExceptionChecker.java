package org.firstinspires.ftc.teamcode.temp;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FailedInitializationException;
import org.firstinspires.ftc.teamcode.RobotVision;

@TeleOp(group = "E")
public class ExceptionChecker extends LinearOpMode {

    RobotVision vision;

    @Override
    public void runOpMode() throws RuntimeException {

        vision = new RobotVision(hardwareMap, telemetry, true, false);

        telemetry.addLine("Ready to crash and burn, Captain!");
        telemetry.update();

        waitForStart();


        try {
            vision.checkExceptionValidity();
        } catch (FailedInitializationException e) {
            telemetry.addLine(e.getMessage());
            telemetry.update();
        }

        sleep(10_000);


    }

}
