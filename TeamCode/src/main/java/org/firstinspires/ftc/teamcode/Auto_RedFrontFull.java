package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
@Autonomous(name = "Red Front", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_RedFrontFull extends CSBase {
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
            drive(-8);
            drive(8);
            turn(30);
        } else if (pos == spike.middle) {
            drive(-10);
            drive(10);
        } else {
            turn(30);
            drive(-8);
            drive(8);
            turn(-30);
        }
        drive(20);
        s(.5);
        drive(-2);
        turn(-90);
        s(3);
        drive(70);
        strafe(5, dir.right);
        setSpeed(1000);
        drive(15);
        setSpeed(2000);
        ejectPixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display final telemetry message.
    }

}
