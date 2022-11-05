package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

@Autonomous(name="WARHOGAuto_testing", group="")
public class WARHOGAuto extends LinearOpMode {
    public WARHOGAuto() throws InterruptedException {}

    private Gamepad GP = new Gamepad();
    private StartPos startPos = null;
    private enum StartPos {
        redLeft,
        redRight,
        blueLeft,
        blueRight,
    };

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        //Intake intake = new Intake(hardwareMap, telemetry);
        Outtake outtake = new Outtake(hardwareMap, telemetry);

        try {
            GP.copy(gamepad1);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        waitForStart();

        //set up startPos
        if (GP.a) {
            startPos = StartPos.redLeft;
        } else if (GP.b) {
            startPos = StartPos.redRight;
        } else if (GP.x) {
            startPos = StartPos.blueLeft;
        } else if (GP.y) {
            startPos = StartPos.blueRight;
        }
        telemetry.addData("Start Position", startPos);
        telemetry.update();

        while(opModeIsActive()) {


            // use camera to detect parking pos
//            telemetry.addData("camera detecting", true);
//            telemetry.update();

            // drive forward
//            drivetrain.MoveForDis(50, 1);
            drivetrain.RotateForDegree(1000, 1);

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
