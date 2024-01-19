package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "Red Back", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_RedBackFull extends CSBase {
    @Override
    public void runOpMode() {
        setup(color.r);

        // ---------------------
        // ------Main Code------
        // ---------------------


        pos = findPos();
        telemetry.addData("Team Prop X", x);
        telemetry.addData("Team Prop Position", pos);
        telemetry.update();
        s(2);
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
        turn(90);
        drive(tilesToInches(1));
        turn(-90);
        drive(tilesToInches(1));
        turn(90);
        detectTag(6);
        drive(tilesToInches(-2.1));
        turn(-90);
        setSpeed(1000);
        drive(tilesToInches(1.7));
        setSpeed(2000);
        ejectPixel();

        //*/

        s(1);  // Pause to display final telemetry message.
    }
}