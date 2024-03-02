package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Base;


//@Autonomous(name = "Class Test", group = "Tests")
@Disabled
public class Test_Class extends Base {
    public void runOpMode() {
        setup(true);
        if (carWashMotor == null) {
            telemetry.addData("CarWashMotor", "null");

        }
        telemetry.update();
        sleep(5);

    }
}
