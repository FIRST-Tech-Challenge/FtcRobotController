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
        turn(-90);
        drive(tiles(1));
        strafeUntilTagDetection(dir.left, ID);
        // Place yellow pixel
        strafeUntilTagDetection(dir.left, 4);
        turn(-90);
        drive(tiles(1));
        turn(90);
        drive(tiles(1));

        //*/

        sleep(1000);  // Pause to display final telemetry message.
    }
}