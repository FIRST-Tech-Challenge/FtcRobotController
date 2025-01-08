package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name = "Bucket Tester")
public class BucketTester extends LinearOpMode {
    private Bucket bucket;
    private GamepadEvents controller;

    @Override
    public void runOpMode() throws InterruptedException {
        bucket = new Bucket(hardwareMap, "bucket");

        while(opModeIsActive())
        {
           double pos = 0;
            if(controller.left_bumper.onPress())
            {
                pos += 0.2;
                if(pos > 1)
                {
                    pos = 0;
                }
                bucket.setPosition(pos);
            }
        }
    }
}
