package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "Red Back", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_RedBackFull extends CSBase {
    @Override
    public void runOpMode() {
        color teamColor = color.red;
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
        turn(90);
        drive(tilesToInches(1.3));
        strafeUntilTagDetection(dir.right, ID);
        // Place yellow pixel
        strafeUntilTagDetection(dir.right, 4);
        turn(90);
        drive(tilesToInches(1));
        turn(90);
        detectTag(6);
        drive(tilesToInches(-2.1));
        turn(-90);
        setSpeed(1000);
        drive(tilesToInches(1.7));
        setSpeed(2000);
        ejectPixel();

        //*/

        s(1);  // Pause to display final telemetry message.
    }
}