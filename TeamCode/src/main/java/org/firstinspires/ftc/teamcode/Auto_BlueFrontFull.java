package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Front Full", group = "CenterStage", preselectTeleOp = "Full")
//@Disabled
public class Auto_BlueFrontFull extends CSBase {
    @Override
    public void runOpMode() {
        stageSide = side.front;
        color teamColor = color.blue;
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
