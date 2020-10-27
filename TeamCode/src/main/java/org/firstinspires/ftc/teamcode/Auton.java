package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous", group="XDrive")
public class Auton extends LinearOpMode {
    DriveTrain bot;
    PID positive, negative, heading;
    double pos, neg, rotation;

    @Override
    public void runOpMode()
    {
        bot.init();
        bot.setControlVariables(pos, neg, rotation);
        positive.setTarget(100);
        negative.setTarget(0);
        heading.setTarget(90);

        waitForStart();

        sleep(1000);

        while (opModeIsActive())
        {
            pos = (positive.runControlLoop(bot.leftFrontDistanceTraveled) + positive.runControlLoop(bot.rightBackDistanceTraveled)) / 2;
            neg = (negative.runControlLoop(bot.leftBackDistanceTraveled) + positive.runControlLoop(bot.rightFrontDistanceTraveled)) / 2;
            rotation = (heading.runControlLoop(bot.leftFrontDistanceTraveled) + heading.runControlLoop(bot.rightFrontDistanceTraveled) + heading.runControlLoop(bot.leftBackDistanceTraveled) + heading.runControlLoop(bot.rightBackDistanceTraveled)) / 4;

            bot.loop();

            if (bot.isTurning) { bot.turn(); }

            if (!bot.isTurning) { bot.move(); }

        }
        bot.stop();
    }
}
