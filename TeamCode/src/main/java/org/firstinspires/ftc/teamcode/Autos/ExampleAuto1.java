package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autos.BASE.AutoBase;

@Autonomous(name = "CoordinateTester")
public class ExampleAuto1 extends AutoBase {

    @Override
    public void autoCode() {
        // Code to do in the autonomous (60.96)
        drive(new double[]{0.62, 0.62});
    }

}
