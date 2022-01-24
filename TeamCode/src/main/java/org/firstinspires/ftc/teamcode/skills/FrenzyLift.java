package org.firstinspires.ftc.teamcode.skills;

import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.bots.FrenzyBot;

public class FrenzyLift implements Runnable {

    private FrenzyBot frenzyBot;
    private FrenzyLiftMode liftMode;
    private String opModeSide = AutoRoute.NAME_RED;
    private boolean score = false; //extend or score
    public FrenzyLift(FrenzyBot bot, String side, FrenzyLiftMode mode, boolean score){
        frenzyBot = bot;
        opModeSide = side;
        liftMode = mode;
    }

    @Override
    public void run() {
        if (score) {
            frenzyBot.scoreAndFold();
        } else {
            if (liftMode.equals(FrenzyLiftMode.SharedHub)) {
                switch (opModeSide) {
                    case AutoRoute.NAME_BLUE:
                        frenzyBot.extendToTeamHubBlue();
                        break;

                    case AutoRoute.NAME_RED:
                        frenzyBot.extendToTeamHubRed();
                        break;
                }
            } else if (liftMode.equals(FrenzyLiftMode.TeamHub)) {
                switch (opModeSide) {
                    case AutoRoute.NAME_BLUE:
                        frenzyBot.dropToTeamHubBlue();
                        break;

                    case AutoRoute.NAME_RED:
                        frenzyBot.dropToTeamHubRed();
                        break;
                }
            }
        }
    }
}
