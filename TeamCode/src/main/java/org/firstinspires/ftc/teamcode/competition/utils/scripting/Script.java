package org.firstinspires.ftc.teamcode.competition.utils.scripting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * A Script represents the code running inside of a LinearOpMode. The reason for using scripts and not writing code directly in the LinearOpMode is to be able to swap and use code between LinearOpModes without keeping multiple copies in each LinearOpMode.
 * @apiNote To use a Script, an OpMode should construct the script before the waitForStart() method, then call script.main() during the main control loop. Call stop() once the loop completes.
 */
public abstract class Script {

    private final LinearOpMode OP_MODE;
    private final ScriptType TYPE;

    /**
     * Makes a new Script.
     *
     * @param opMode The OpMode the Script is being run in.
     * @param type The ScriptType of the Script.
     */
    public Script(LinearOpMode opMode, ScriptType type) {
        OP_MODE = opMode;
        TYPE = type;
    }

    public LinearOpMode getOpMode() {
        return OP_MODE;
    }

    public String toString() {
        return "Script: " + getType() + "-" + getName();
    }

    public abstract void main();

    public abstract void stop();

    public abstract String getName();

    public ScriptType getType() {
        return TYPE;
    }

}
