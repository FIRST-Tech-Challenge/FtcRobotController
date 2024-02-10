package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "Red Back 50", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_RedBackFull extends CSBase {
    @Override
    public void runOpMode() {
        color teamColor = color.r;
        stageSide = side.b;
        setup(teamColor);

        // ---------------------
        // ------Main Code------
        // ---------------------


        pos = findPos();
        int ID = setID(pos, teamColor);
        telemetry.addData("Team Prop X", x);
        telemetry.addData("Team Prop Position", pos);
        telemetry.update();
        purplePixel();
        drive(-2);
        turn(90, dir.r);
        drive(tilesToInches(-1));
//        turn(-90);
//        drive(tilesToInches(-1));
        setSpeed(1000);
        for (int i = 0; i < 6; i++) {
            telemetry.addData("i", i);
            telemetry.update();
            strafe(5, dir.r);
            if (tagDetections(ID, 1000) != null) {
                break;
            }
        }
        align(ID);
        //moveLift(3);
        //drive(-12);
        //strafe(tilesToInches(1), dir.r);
        //drive(-18);

        //*/

        s(1);  // Pause to display final telemetry message.
    }
}