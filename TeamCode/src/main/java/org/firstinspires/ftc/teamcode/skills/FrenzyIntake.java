package org.firstinspires.ftc.teamcode.skills;

import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.bots.FrenzyBot;

public class FrenzyIntake implements Runnable {

    private FrenzyBot frenzyBot;
    public FrenzyIntake(FrenzyBot bot){
        frenzyBot = bot;
    }

    @Override
    public void run() {
        frenzyBot.smartStopIntake();
    }
}
