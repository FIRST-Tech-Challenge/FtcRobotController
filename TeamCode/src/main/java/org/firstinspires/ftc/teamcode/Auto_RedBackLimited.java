package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Back 28", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_RedBackLimited extends CSBase {
    @Override
    public void runOpMode() {
        stageSide = side.b;
        setup(color.r);

        // ---------------------
        // ------Main Code------
        // ---------------------

        pos = findPos();
//        int ID = setID(pos, teamColor);
        telemetry.addData("Team Prop X", x);
        telemetry.addData("Team Prop Position", pos);
        telemetry.update();
        purplePixel();
        drive(-2);
        turn(90);
        drive(tilesToInches(-1));
        turn(90);
        drive(tilesToInches(2));
        turn(90);
        setSpeed(1000);
        drive(tilesToInches(0.4));
        ejectPixel();
        drive(tilesToInches(0.1));

        //moveLift(3);
        //drive(-12);
        //strafe(tilesToInches(1), dir.r);
        //drive(-18);

        //*/


//        drive(tilesToInches(-2.1));
//        turn(-90);
//        setSpeed(1000);
//        drive(tilesToInches(1.7));
//        setSpeed(2000);
//        ejectPixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        s(1);  // Pause to display final telemetry message.
    }
}