package org.firstinspires.ftc.teamcode.archive.CenterStage;import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "Nothing", group = "CenterStage", preselectTeleOp = "Main")
public class Auto_Nothing extends LinearOpMode {
    public void runOpMode() {
        telemetry.addData("Doing", "Nothing");
        telemetry.update();
        waitForStart();
        while (opModeIsActive());
    }
}
