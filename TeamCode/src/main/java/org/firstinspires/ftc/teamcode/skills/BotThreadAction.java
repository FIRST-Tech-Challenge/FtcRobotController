package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;


public class BotThreadAction implements Runnable {
    private Telemetry telemetry;
    private LinearOpMode caller = null;
    private UltimateBot robot = null;
    private String function = "";
    private boolean isRunning = true;

    public BotThreadAction(UltimateBot bot, Telemetry telemetry, String function, LinearOpMode caller) {
        robot = bot;
        this.telemetry = telemetry;
        this.caller = caller;
        this.function = function;
    }

    @Override
    public void run() {
        while (isRunning) {
            if (function.contains("wallclose")) {
                robot.liftWallGrab();
                break;
            } else if (function.contains("wobbleback")) {
                robot.backWobbleSwing();
                break;
            } else if (function.contains("wobbleforward")) {
                robot.forwardWobbleSwing();
                break;
            } else if (function.contains("wobblewall")) {
                robot.liftWobbleWall();
                break;
            }
        }
    }
}
