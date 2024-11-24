package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotSubSystems.Camera.Camera;
import org.firstinspires.ftc.teamcode.robotSubSystems.Camera.TestPipeline;

@Autonomous
public class Testimgproc extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TestPipeline pipeline = new TestPipeline();
        Camera.init(hardwareMap);
        waitForStart();
        Camera.init(hardwareMap);
    }
}
