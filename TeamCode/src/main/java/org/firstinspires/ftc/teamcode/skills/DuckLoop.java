package org.firstinspires.ftc.teamcode.skills;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.FrenzyBot;

public class DuckLoop implements Runnable {

    private final String TAG = "DuckLoopThread";

    private FrenzyBot frenzyBot;
    private boolean redSide;
    private boolean isAuto;

    public DuckLoop(FrenzyBot frenzyBot, boolean isRed, boolean auto) {
        this.frenzyBot = frenzyBot;
        this.redSide = isRed;
        this.isAuto = auto;

        Log.d(TAG, String.format("Initialized DuckLoop Skill. red=%b, auto=%b", isRed, auto));
    }

    @Override
    public void run() {
        frenzyBot.duckLoop(this.redSide, this.isAuto);
    }
}
