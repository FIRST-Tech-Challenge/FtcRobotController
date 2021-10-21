package org.firstinspires.ftc.teamcode.Tests.TeleopTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.io.IOException;

@TeleOp(name="Detect Marker Test")
public class DetectMarkerTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode()
    {

        try {
            Robot robot = new Robot(this, timer);
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
