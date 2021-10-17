package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.io.IOException;


@Autonomous(name = "Auto", group = "Concept")
@Disabled
public class Auto extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    private Robot robot;

    {
        try {
            robot = new Robot(this, timer);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}
