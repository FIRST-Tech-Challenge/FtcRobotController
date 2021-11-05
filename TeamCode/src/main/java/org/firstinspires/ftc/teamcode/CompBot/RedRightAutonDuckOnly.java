package org.firstinspires.ftc.teamcode.CompBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Right Autonomous, Duck Only",group="CompBot")
@Disabled
public class RedRightAutonDuckOnly extends LinearOpMode {
    CompBotHW r = new CompBotHW();

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        waitForStart();

        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds()<2000) {
            r.m.driveRobotCentric(0.5,0,0.2);
        }
        r.m.stop();
        e.reset();
        while(e.milliseconds()<5000) {
            r.spin.set(0.2);
        }
        r.spin.set(0);
        e.reset();
        while(e.milliseconds() < 6000) {
            r.m.driveRobotCentric(1,0,0);
        }
        e.reset();
        while(!isStopRequested()) {
            r.m.driveRobotCentric(0.1,0,0);
        }

    }
}
