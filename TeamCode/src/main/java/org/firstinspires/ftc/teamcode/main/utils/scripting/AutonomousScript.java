package org.firstinspires.ftc.teamcode.main.utils.scripting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutonomousScript extends Script {

    /**
     * Makes a new Script.
     *
     * @param opMode The OpMode the Script is being run in.
     */
    public AutonomousScript(LinearOpMode opMode) {
        super(opMode, ScriptType.AUTONOMOUS);
    }

}
