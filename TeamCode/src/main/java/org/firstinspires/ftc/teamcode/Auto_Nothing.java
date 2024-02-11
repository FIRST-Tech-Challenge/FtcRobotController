package org.firstinspires.ftc.teamcode;import com.qualcomm.robotcore.eventloop.opmode.*;
@Autonomous(name = "Nothing", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_Nothing extends LinearOpMode {
    public void runOpMode() {
        telemetry.addData("Doing", "Nothing");
        telemetry.update();
        waitForStart();
        while (opModeIsActive());
    }
}
