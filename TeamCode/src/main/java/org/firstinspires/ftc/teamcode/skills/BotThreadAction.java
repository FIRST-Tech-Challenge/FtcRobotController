package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;


public class BotThreadAction implements Runnable {
    private Telemetry telemetry;
    private LinearOpMode caller = null;
    private UltimateBot robot = null;
    private String function = "";

    public BotThreadAction(UltimateBot bot, Telemetry telemetry, String function, LinearOpMode caller) {
        robot = bot;
        this.telemetry = telemetry;
        this.caller = caller;
        this.function = function;
    }

    @Override
    public void run() {
        if (function.contains("wallclose")) {
            robot.liftWallGrab();
        } else if (function.contains("wobbleback")) {
            robot.backWobbleSwing();
        } else if (function.contains("wobbleforward")) {
            robot.forwardWobbleSwing();
        } else if (function.contains("wobblewall")) {
            robot.liftWobbleWall();
        }
    }
}
