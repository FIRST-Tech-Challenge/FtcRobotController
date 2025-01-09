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
        controller = new GamepadEvents(gamepad1);
        double pos = 0;

        telemetry.addLine("Initalized");
        waitForStart();
        while(opModeIsActive())
        {
            if(controller.left_bumper.onPress())
            {
                bucket.setUp();
                telemetry.addData("Bucket Pos", bucket.getPosition());
            }else if(controller.right_bumper.onPress())
            {
                bucket.setBottom();
                telemetry.addData("Bucket Pos", bucket.getPosition());
            }
            telemetry.update();
            controller.update();
        }

    }
}
