package org.firstinspires.ftc.teamcode.McDonald;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class HelloLanden extends OpMode {
    @Override
    public void init() {telemetry.addData("Hello","Landen");
    }
// Added caption Hello
    @Override
    public void loop() {
   // Nothing was added to the public void loop
    }
}
