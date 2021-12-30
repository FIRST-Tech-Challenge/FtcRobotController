package org.firstinspires.ftc.teamcode.main.opmodes.production;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.main.scripts.teleop.FullTeleOpScript;
import org.firstinspires.ftc.teamcode.main.utils.scripting.Script;
import org.firstinspires.ftc.teamcode.main.utils.scripting.ScriptRunner;

@TeleOp(name="FullTeleOp", group="production")
public class FullTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Script script = new FullTeleOpScript(this);
        new ScriptRunner(this, script);
    }

}
