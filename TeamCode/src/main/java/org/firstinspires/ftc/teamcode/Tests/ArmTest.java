package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Robots.BradBot;
@Disabled

@Autonomous
@Config
public class ArmTest extends RFServoTest{

    public static double LOAD = 0.28, SHOOT = 0.98;
    public static int target = 0;
    private int atTarg = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize("armServo", 0.5, LOAD, SHOOT);
        waitForStart();
        while (opModeIsActive()) {
            double[] pussitions = {LOAD, SHOOT};

            if (target != atTarg) {
                flipTo(pussitions[target]);
                atTarg = target;
            }
            packet.put("atTarg", atTarg);
            robot.update();
        }
    }
}
