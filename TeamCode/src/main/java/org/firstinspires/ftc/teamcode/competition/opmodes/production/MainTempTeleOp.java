package org.firstinspires.ftc.teamcode.competition.opmodes.production;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.scripts.teleop.MainTempTeleOpScript;
import org.firstinspires.ftc.teamcode.competition.utils.scripting.Script;
import org.firstinspires.ftc.teamcode.competition.utils.scripting.ScriptRunner;
import org.firstinspires.ftc.teamcode.competition.utils.scripting.TeleOpStubScript;

@TeleOp(name="MainTempTeleOp", group="production")
public class MainTempTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Script script = new MainTempTeleOpScript(this);
        new ScriptRunner(this, script);
    }

}
