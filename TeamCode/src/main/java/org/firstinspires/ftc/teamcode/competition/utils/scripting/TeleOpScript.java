package org.firstinspires.ftc.teamcode.competition.utils.scripting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class TeleOpScript extends Script {

    /**
     * Makes a new Script.
     *
     * @param opMode The OpMode the Script is being run in.
     */
    public TeleOpScript(LinearOpMode opMode) {
        super(opMode, ScriptType.TELE_OP);
    }

}
