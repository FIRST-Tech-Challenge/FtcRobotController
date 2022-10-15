package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Drivetrain;

@Autonomous(name="WARHOGAuto", group="")
public class WARHOGAuto extends LinearOpMode {

//    private Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
//    private Intake intake = new Intake(hardwareMap, telemetry);
    private StartPos startPos = null;

    public WARHOGAuto() throws InterruptedException {
    }

    private enum StartPos {
        redLeft,
        redRight,
        blueLeft,
        blueRight,
    };

    @Override
    public void runOpMode() throws InterruptedException {

        //set up startPos
//        Gamepad gamepad = gamepad1;
        if (gamepad1.a) {
            startPos = StartPos.redLeft;
        } else if (gamepad1.b) {
            startPos = StartPos.redRight;
        } else if (gamepad1.x) {
            startPos = StartPos.blueLeft;
        } else if (gamepad1.y) {
            startPos = StartPos.blueRight;
        }

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("start position", startPos);
            telemetry.update();
        }
    }
}
