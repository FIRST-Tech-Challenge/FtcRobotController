package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class Controller {

    public Controller(){}

    public void rumble() {
        gamepad1.rumble(100);
    }
}
