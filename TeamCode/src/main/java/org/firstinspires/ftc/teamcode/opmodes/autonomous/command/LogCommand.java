package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

public class LogCommand extends SounderBotCommandBase {

    private final String tag;
    private final String msg;

    public LogCommand(String tag, String msg) {
        this.tag = tag;
        this.msg = msg;
    }

    @Override
    protected void doExecute() {
        Log.i(tag, msg);
        finished = true;
    }

    @Override
    protected boolean isTargetReached() {
        return false;
    }
}
