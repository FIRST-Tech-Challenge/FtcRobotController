package org.firstinspires.ftc.teamcode.archive.CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Base;

//@Autonomous(name = "Red Back 50", group = "CenterStage", preselectTeleOp = "Main")
@Disabled
public class Auto_RedBackFull extends Base {
    @Override
    public void runOpMode() {
        color teamColor = color.red;
        stageSide = side.back;
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
        turn(90);
        drive(tilesToInches(-1));
//        turn(-90);
//        drive(tilesToInches(-1));
        setSpeed(1000);
        strafe(15, dir.right);
        for (int i = 0; i < 3; i++) {
            telemetry.addData("Strafe #", i);
            telemetry.update();
            if (tagDetections(ID, 1000) != null) {
                break;
            }
            strafe(5, dir.right);
        }
        align(ID);
        moveLift(GOAL_ENCODERS);
        dropPixels();
        retractLift();
        //drive(-12);
        //strafe(tilesToInches(1), dir.r);
        //drive(-18);

        //*/

        s(1);  // Pause to display final telemetry message.
    }
}