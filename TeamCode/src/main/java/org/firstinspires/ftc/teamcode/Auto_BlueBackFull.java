package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

//@Autonomous(name = "Blue Back 50", group = "CenterStage", preselectTeleOp = "Full")
@Disabled
public class Auto_BlueBackFull extends CSBase {

    @Override
    public void runOpMode() {
        stageSide = side.back;
        color teamColor = color.blue;
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
        turn(-90, dir.left);
        drive(tilesToInches(-1));
//        turn(-90);
//        drive(tilesToInches(-1));
        setSpeed(1000);
        strafe(15, dir.left);
        for (int i = 0; i < 3; i++) {
            telemetry.addData("Strafe #", i);
            telemetry.update();
            if (tagDetections(ID, 1000) != null) {
                break;
            }
            strafe(5, dir.left);
        }
        align(ID);
        moveLift(GOAL_ENCODERS);
        dropPixels();
        retractLift();

//        findPos();
//        purplePixel();
//        drive(tilesToInches(-2.1));
//        turn(90);
//        setSpeed(1000);
//        drive(tilesToInches(1.7));
//        setSpeed(2000);
//        ejectPixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display final telemetry message.
    }

}