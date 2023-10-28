package org.firstinspires.ftc.teamcode.mcanally;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
@Disabled
public class UsestringLBM extends OpMode{
@Override
    public void init()  {
        String myName = "Luke McAnally";

        telemetry.addData("Hello", myName);
    }

    @Override
    public void loop() {

    }
}
