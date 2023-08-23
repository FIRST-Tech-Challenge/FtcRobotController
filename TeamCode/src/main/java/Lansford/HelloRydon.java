package Lansford;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class HelloRydon extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","Rydon");
    }

    @Override
    public void loop() {

    }
}
