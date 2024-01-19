package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Front Final", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_BlueFrontLimited extends CSBase {
    @Override
    public void runOpMode() {
        setup(color.b);

        // ---------------------
        // ------Main Code------
        // ---------------------

        drive(-2);
        turn(90);
        sleep(3);
        drive(70);
        setSpeed(1000);
        drive(15);
        setSpeed(2000);
        ejectPixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display final telemetry message.
    }
}