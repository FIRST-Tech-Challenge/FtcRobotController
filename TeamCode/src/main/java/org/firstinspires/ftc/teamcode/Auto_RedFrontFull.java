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
        purplePixel();
        drive(-2);
        turn(-90);
        s(3);
        drive(70);
        strafe(5, dir.right);
        setSpeed(1000);
        drive(15);
        setSpeed(2000);
        ejectPixel();
        drive(5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display final telemetry message.
    }

}
