package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Front Full", group = "CenterStage", preselectTeleOp = "Full")
//@Disabled
public class Auto_BlueFrontFull extends CSBase {
    @Override
    public void runOpMode() {
        stageSide = side.f;
        color teamColor = color.b;
        setup(teamColor);

        // ---------------------
        // ------Main Code------
        // ---------------------

        s(1);
        pos = findPos();
        int ID = setID(pos, teamColor);
        telemetry.addData("Team Prop X", x);
        telemetry.addData("Team Prop Position", pos);
        telemetry.update();
        purplePixel();
        drive(-2);
        turn(-90);
        s(2);
        drive(-70);
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
//        drive(-2);
//        turn(90, dir.r);
//        s(3);
//        drive(70);
//        setSpeed(1000);
//        drive(15);
//        setSpeed(2000);
//        ejectPixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        s(1);  // Pause to display final telemetry message.
    }


}
