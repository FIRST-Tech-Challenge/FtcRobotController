package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Field-Centric TeleOp")
public class TeleOpController extends OpMode {
    TeleOpDriver driveTrain = new TeleOpDriver();

    Boolean LastCheck = Boolean.FALSE;
    Boolean Current = Boolean.FALSE;

    @Override
    public void init() {

        driveTrain.intialiseDriver();
    }

    @Override
    public void loop() {
        driveTrain.DriveLoop();
        Current = gamepad1.y;
        if (Current & !LastCheck) {
            driveTrain.resetHeading();
        }
        LastCheck = Current;

    }
}
