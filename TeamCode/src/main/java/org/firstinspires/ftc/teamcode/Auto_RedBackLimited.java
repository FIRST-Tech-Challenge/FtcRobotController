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

        drive(tilesToInches(-2.1));
        turn(-90);
        setSpeed(1000);
        drive(tilesToInches(1.7));
        setSpeed(2000);
        ejectPixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        s(1);  // Pause to display final telemetry message.
    }
}