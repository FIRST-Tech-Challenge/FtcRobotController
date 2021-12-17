package org.firstinspires.ftc.teamcode.competition.opmodes.templates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.utils.scripting.AutonomousStubScript;
import org.firstinspires.ftc.teamcode.competition.utils.scripting.ControlLoopManager;
import org.firstinspires.ftc.teamcode.competition.utils.scripting.Script;
import org.firstinspires.ftc.teamcode.competition.utils.scripting.ScriptRunner;
import org.firstinspires.ftc.teamcode.competition.utils.scripting.TeleOpStubScript;

@TeleOp(name="TeleOpTemplate", group="templates") // replace the name and group with your OpMode's name and group
public class LinearTeleOpTemplate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Script script = new TeleOpStubScript(this); // replace this with your Script
        new ScriptRunner(this, script);
    }

}
