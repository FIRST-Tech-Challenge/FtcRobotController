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
        sleep(5000);
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
        drive(-2);
        turn(-90);
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
