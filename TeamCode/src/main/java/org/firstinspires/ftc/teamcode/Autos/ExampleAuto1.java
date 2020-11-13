package org.firstinspires.ftc.teamcode.Autos;

import org.firstinspires.ftc.teamcode.Autos.BASE.AutoBase;

public class ExampleAuto1 extends AutoBase {

    @Override
    public void autoCode() {
        // Code to do in the autonomous
        drive(0.8, 1.0);
        turn(0.8, -90);
        drive(0.5, -0.5);
    }

}
