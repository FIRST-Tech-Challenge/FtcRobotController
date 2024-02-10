package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "Blue Back Full", group = "CenterStage", preselectTeleOp = "Full")
//@Disabled
public class Auto_BlueBackFull extends CSBase {

    @Override
    public void runOpMode() {
        stageSide = side.b;
        color teamColor = color.b;
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
        turn(-90, dir.l);
        drive(tilesToInches(-1));
//        turn(-90);
//        drive(tilesToInches(-1));
        setSpeed(1000);
        for (int i = 0; i < 6; i++) {
            telemetry.addData("i", i);
            telemetry.update();
            strafe(5, dir.l);
            if (tagDetections(ID, 1000) != null) {
                break;
            }
        }
        align(ID);

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