package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "Red Back", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_RedBackFull extends CSBase {
    @Override
    public void runOpMode() {
        setup(color.red);

        // ---------------------
        // ------Main Code------
        // ---------------------


        pos = findPos();
        telemetry.addData("Team Prop X", x);
        telemetry.addData("Team Prop Position", pos);
        telemetry.update();
        s(2);
        drive(-16);
        if (pos == spike.left) {
            turn(-30);
            drive(-9);
            drive(9);
            turn(30);
        } else if (pos == spike.middle) {
            drive(-10);
            drive(10);
        } else {
            turn(30);
            drive(-9);
            drive(9);
            turn(-30);
        }
        turn(-90);
        drive(tiles(1));
        turn(-90);
        drive(tiles(1));
        turn(90);
        detectTag(6);
        drive(tiles(-2.1));
        turn(-90);
        setSpeed(1000);
        drive(tiles(1.7));
        setSpeed(2000);
        ejectPixel();

        //*/

        sleep(1000);  // Pause to display final telemetry message.
    }
}