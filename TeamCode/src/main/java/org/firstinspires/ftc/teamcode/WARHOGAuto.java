package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Drivetrain;

@Autonomous(name="WARHOGAuto_testing", group="")
public class WARHOGAuto extends LinearOpMode {

    private Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
//    private Intake intake = new Intake(hardwareMap, telemetry);
    private StartPos startPos = null;
    private enum StartPos {
        redLeft,
        redRight,
        blueLeft,
        blueRight,
    };

    public WARHOGAuto() throws InterruptedException {
    }


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        //set up startPos
        if (gamepad1.a) {
            startPos = StartPos.redLeft;
        } else if (gamepad1.b) {
            startPos = StartPos.redRight;
        } else if (gamepad1.x) {
            startPos = StartPos.blueLeft;
        } else if (gamepad1.y) {
            startPos = StartPos.blueRight;
        }
        telemetry.addData("Start Position", startPos);
        telemetry.update();

        while(opModeIsActive()) {


            // use camera to detect parking pos
            telemetry.addData("camera detecting", true);
            telemetry.update();

            // drive forward
            drivetrain.MoveForDis(100, 0.5);

            // putting cone on pole


            // turn to cone stack

            // move forward

            // take another cone

            // turn back

            // drive forward

            // putting cone on pole

            // park

            // end this part
            break;
        }
    }
}
