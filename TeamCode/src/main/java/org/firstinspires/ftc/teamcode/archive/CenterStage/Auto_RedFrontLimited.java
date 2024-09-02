package org.firstinspires.ftc.teamcode.archive.CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Base;


//@Autonomous(name = "Red Front 28", group = "CenterStage", preselectTeleOp = "Main")
@Disabled
public class Auto_RedFrontLimited extends Base {
    @Override
    public void runOpMode() {
        stageSide = side.front;
        setup(color.red);

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
        turn(-90);
        s(2);
        drive(70);
        setSpeed(1000);
        drive(15);
        setSpeed(2000);
        ejectPixel(3000);
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
