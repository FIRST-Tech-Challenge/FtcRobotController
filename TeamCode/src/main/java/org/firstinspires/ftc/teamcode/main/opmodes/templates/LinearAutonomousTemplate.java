package org.firstinspires.ftc.teamcode.main.opmodes.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.utils.scripting.AutonomousStubScript;
import org.firstinspires.ftc.teamcode.main.utils.scripting.Script;
import org.firstinspires.ftc.teamcode.main.utils.scripting.ScriptRunner;

@Disabled // remove this annotation
@Autonomous(name="AutonomousTemplate", group="templates") // replace the name and group with your OpMode's name and group
public class LinearAutonomousTemplate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Script script = new AutonomousStubScript(this); // replace this with your Script
        new ScriptRunner(this, script);
    }

}
