package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class TestAuto extends LinearOpMode {

    public PARKING_POSITION parkingposition = PARKING_POSITION.BOARD;
    public enum PARKING_POSITION {
        FREEWAY, TRUSS, BOARD
    }
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true, true, false);
        robot.setUpDrivetrainMotors();
        //.setUpIntakeOuttake();
        robot.initVisionProcessing();

        double waitCounter = 0;
        boolean wasPressedB = false;
        boolean wasPressedX = false;
        boolean wasPressedDPadLeft = false;
        boolean wasPressedDPadRight = false;
        boolean wasPressedDPadUp = false;

        while (!opModeIsActive()) {

            // b increases wait counter by 1
            if (wasPressedB == true && !gamepad1.b) {
                waitCounter++;
            }
            wasPressedB = gamepad1.b;

            // x decreases wait counter by 1
            if (wasPressedX == true && !gamepad1.x) {
                waitCounter--;
            }
            wasPressedX = gamepad1.x;

            // wait counter cannot be negative
            if (waitCounter < 0) {
                waitCounter = 0;
            }

            // dpad left - park left
            if (wasPressedDPadLeft == true && !gamepad1.dpad_left) {
                parkingposition = PARKING_POSITION.FREEWAY;
            }
            wasPressedDPadLeft = gamepad1.dpad_left;

            // dpad right - park right
            if (wasPressedDPadRight == true && !gamepad1.dpad_right) {
                parkingposition = PARKING_POSITION.TRUSS;
            }
            wasPressedDPadRight = gamepad1.dpad_right;

            // dpad up - don't park
            if (wasPressedDPadUp == true && !gamepad1.dpad_up) {
                parkingposition = PARKING_POSITION.BOARD;
            }
            wasPressedDPadUp = gamepad1.dpad_up;

            if(gamepad1.left_bumper == true && gamepad1.right_bumper == true){
                telemetry.addLine("CONFIRMED");
                telemetry.addLine("");

                telemetry.addLine("+/- WAIT: B/X");
                telemetry.addData("WAIT TIME: ", waitCounter);
                telemetry.addLine("");

                telemetry.addLine("PARKING: DPAD LEFT/UP/RIGHT");
                telemetry.addData("PARKING POS: ", parkingposition);
                telemetry.update();
                break;
            }

            telemetry.addLine("+/- WAIT: B/X");
            telemetry.addData("WAIT TIME: ", waitCounter);
            telemetry.addLine("");

            telemetry.addLine("PARKING: DPAD LEFT/UP/RIGHT");
            telemetry.addData("PARKING POS: ", parkingposition);
            telemetry.addLine("");

            telemetry.addLine("PRESS BOTH BUMPERS TO CONFIRM");

            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            //robot.moveLinearSlideByTicksBlocking(-2000);
            //robot.trayToOuttakePos(true);
            this.sleep(1000);

            //robot.trayToIntakePos(true);
            this.sleep(2500);

            //robot.mecanumAndSlidesDownToZero(-24);
            break;
        }
    }
}

