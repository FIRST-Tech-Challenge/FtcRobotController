package org.firstinspires.ftc.teamcode.main.opmodes.production;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.main.scripts.teleop.MainTempTeleOpScript;
import org.firstinspires.ftc.teamcode.main.utils.scripting.Script;
import org.firstinspires.ftc.teamcode.main.utils.scripting.ScriptRunner;
import org.firstinspires.ftc.teamcode.main.utils.scripting.TeleOpStubScript;

@TeleOp(name="FullTeleOp", group="production")
public class FullTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Script script = new MainTempTeleOpScript(this);
        new ScriptRunner(this, script);
    }

}
