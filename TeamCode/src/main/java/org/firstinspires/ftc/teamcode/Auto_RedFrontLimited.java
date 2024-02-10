package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Red Front 28", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_RedFrontLimited extends CSBase {
    @Override
    public void runOpMode() {
        stageSide = side.f;
        setup(color.r);

        // ---------------------
        // ------Main Code------
        // ---------------------

        s(1);
        pos = findPos();
//        int ID = setID(pos, teamColor);
        telemetry.addData("Team Prop X", x);
        telemetry.addData("Team Prop Position", pos);
        telemetry.update();
        purplePixel();
        drive(-2);
        turn(-90, dir.l);
        s(3);
        drive(70);
        setSpeed(1000);
        drive(15);
        setSpeed(2000);
        ejectPixel();
        drive(5);

//        pos = findPos();
//        telemetry.addData("Team Prop X", x);
//        telemetry.addData("Team Prop Position", pos);
//        telemetry.update();
//        purplePixel();
//        drive(-2);
//        turn(-90);
//        s(3);
//        drive(70);
//        setSpeed(1000);
//        drive(15);
//        setSpeed(2000);
//        ejectPixel();
//        drive(5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        s(1);  // Pause to display final telemetry message.
    }
}
