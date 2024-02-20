package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class TestAuto extends LinearOpMode {

    public PARKING_POSITION parkingposition = PARKING_POSITION.BOARD;
    public enum PARKING_POSITION {
        LEFT, RIGHT, BOARD
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
            if(wasPressedB == true && !gamepad1.b) {
                waitCounter++;
            }
            wasPressedB = gamepad1.b;

            if(wasPressedX == true && !gamepad1.x) {
                waitCounter--;
            }
            wasPressedX = gamepad1.x;

            if(waitCounter<0){
                waitCounter = 0;
            }


            if(wasPressedDPadLeft == true && !gamepad1.dpad_left) {
                parkingposition = PARKING_POSITION.LEFT;
            }
            wasPressedDPadLeft = gamepad1.dpad_left;

            if(wasPressedDPadRight == true && !gamepad1.dpad_right) {
                parkingposition = PARKING_POSITION.RIGHT;
            }
            wasPressedDPadRight = gamepad1.dpad_right;

            if(wasPressedDPadUp == true && !gamepad1.dpad_up) {
                parkingposition = PARKING_POSITION.BOARD;
            }
            wasPressedDPadUp = gamepad1.dpad_up;

            telemetry.addLine("+/- WAIT: B/X");
            telemetry.addData("WAIT TIME: ", waitCounter);
            telemetry.addLine("");

            telemetry.addLine("PARKING: DPAD LEFT/UP/RIGHT");
            telemetry.addData("PARKING POS: ", parkingposition);
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

