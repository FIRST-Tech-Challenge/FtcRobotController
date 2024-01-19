package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Autonomous(name = "Blue Front", group = "CenterStage", preselectTeleOp = "Full")
@Disabled
public class Auto_BlueFrontFull extends CSBase {
    @Override
    public void runOpMode() {
        setup(color.blue);

        // ---------------------
        // ------Main Code------
        // ---------------------


        findPos();
        drive(-3);
        if (pos == spike.l) {
            turn(-10);
            drive(-20);
            drive(20);
            turn(10);
            drive(3);
        } else if (pos == spike.m) {
            drive(-20);
            drive(23);
        } else {
            turn(10);
            drive(-20);
            drive(20);
            turn(-10);
            drive(3);
        }
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
