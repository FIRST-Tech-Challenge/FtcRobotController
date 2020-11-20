package org.firstinspires.ftc.teamcode.frontend.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.middleend.bases.AutoBase;

@Autonomous(name = "CoordinateTester")
public class ExampleAuto1 extends AutoBase {

    @Override
    public void autoCode() {
        // Code to do in the autonomous (60.96)
        drive(new double[]{0.62, 0.62});
    }

}
