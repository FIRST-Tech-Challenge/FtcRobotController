package org.firstinspires.ftc.teamcode.UltimateGoalOpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RingDetector;

@Autonomous(name="Test")
public class Red extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RingDetector detector = new RingDetector(this);

        waitForStart();

        int rings = detector.getDecision();
    }
}
