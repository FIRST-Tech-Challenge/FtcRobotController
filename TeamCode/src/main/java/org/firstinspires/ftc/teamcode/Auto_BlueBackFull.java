package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Autonomous(name = "Blue Back", group = "CenterStage", preselectTeleOp = "Full")
@Disabled
public class Auto_BlueBackFull extends CSBase {

    @Override
    public void runOpMode() {
        setup(color.blue);

        // ---------------------
        // ------Main Code------
        // ---------------------


        findPos();
        drive(-3);
        if (pos == spike.left) {
            turn(-10);
            drive(-20);
            drive(20);
            turn(10);
            drive(3);
        } else if (pos == spike.middle) {
            drive(-20);
            drive(23);
        } else {
            turn(10);
            drive(-20);
            drive(20);
            turn(-10);
            drive(3);
        }
        drive(tiles(-2.1));
        turn(90);
        setSpeed(1000);
        drive(tiles(1.7));
        setSpeed(2000);
        ejectPixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display final telemetry message.
    }

}