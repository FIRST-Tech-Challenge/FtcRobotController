package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Back Final", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_RedBackLimited extends CSBase {
    @Override
    public void runOpMode() {
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
        sleep(1000);  // Pause to display final telemetry message.
    }
}