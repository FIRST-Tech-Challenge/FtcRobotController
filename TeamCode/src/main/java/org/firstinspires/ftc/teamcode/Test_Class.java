package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;


//@Autonomous(name = "Class Test", group = "Tests")
@Disabled
public class Test_Class extends CSBase {
    public void runOpMode() {
        setup(true);
        if (carWashMotor == null) {
            telemetry.addData("CarWashMotor", "null");

        }
        telemetry.update();
        sleep(5);

    }
}
