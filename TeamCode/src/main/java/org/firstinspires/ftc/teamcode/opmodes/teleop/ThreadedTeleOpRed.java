package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.LightRed;
import org.firstinspires.ftc.teamcode.hardware.Lights;


//threaded tele op controller......
@TeleOp(name = "TeleOp-Red")
public class ThreadedTeleOpRed extends ThreadedTeleOpBase {
    @Override
    public Lights getLights() {
        return new LightRed(hardwareMap.dcMotor.get("LIGHTS"));
    }
}
