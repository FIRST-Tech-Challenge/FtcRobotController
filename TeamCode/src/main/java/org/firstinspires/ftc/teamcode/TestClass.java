package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp()
public class TestClass extends OpMode{
    private Telemetry.Item output = null;
    EncooderTransferClass TransferClass = new EncooderTransferClass("test.txt");
    @Override
    public void init() {
        output = telemetry.addData("output", "");
        TransferClass.runProcess(500);

    }
    @Override
    public void loop() {
        output.setValue(TransferClass.readEncoderPos());
    }


}
