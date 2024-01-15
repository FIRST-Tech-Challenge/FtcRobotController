package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.DriveColorExample;
import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.LedLights;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.RoadRunnerDriving;

@TeleOp(name="Competition drive", group="Linear Opmode")
public class MainDriveProgram extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    boolean canToggleSlowMode = true;
    boolean canToggleHandPosition = true;
    boolean reverseIntake = false;

    boolean inEndGame = false;

    @Override
    public void runOpMode() {

        // create instance of drive class
        RoadRunnerDriving rrDrive = new RoadRunnerDriving(hardwareMap);
        // create instance of lift arm class
        LiftArm arm = new LiftArm(hardwareMap);
        // create instance of intake system class
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap);

        //LedLights leds = new LedLights(hardwareMap);

        DriveColorExample colorSensors = new DriveColorExample(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //leds.setLed(LedLights.ledStates.BLUE);


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            if (!inEndGame && getRuntime() > 90) {
                inEndGame = true;
                //leds.setLed(LedLights.ledStates.ENDGAME);
            }

            /**
             * Driving
             */

            rrDrive.updateMotorsFromStick(gamepad1);
            rrDrive.update();



            /**
             * Intake
             */
            if ((colorSensors.getBackPixelColor() != DriveColorExample.Colors.NONE) && (colorSensors.getFrontPixelColor() != DriveColorExample.Colors.NONE)) {
                reverseIntake = true;
            } else {
                reverseIntake = false;
            }

            if ((gamepad2.right_trigger > 0.5 || gamepad1.right_trigger > 0.5)) {
                if (reverseIntake) {
                    intakeSystem.runIntakeSystem(-1);
                } else {
                    intakeSystem.runIntakeSystem(1);
                }
            } else if ((gamepad2.left_trigger > 0.5 || gamepad1.left_trigger > 0.5)) {
                intakeSystem.runIntakeSystem(-0.5);
            } else {
                intakeSystem.runIntakeSystem(0);
            }



            /**
             * Arm slides
             */
            if (gamepad2.dpad_up){
                arm.extend(LiftArm.Distance.FULL);
                arm.holdArm();
            } else if (gamepad2.dpad_right){
                arm.extend(LiftArm.Distance.HALF);
                arm.holdArm();
            } else if (gamepad2.dpad_left) {
                arm.extend(LiftArm.Distance.AUTO);
                arm.holdArm();
            } else if (gamepad2.dpad_down) {
                arm.retract();
            }



            /**
             * Trapdoor
             */
            if (gamepad2.right_bumper) {
                arm.openTrapdoor();
            } else {
                arm.closeTrapdoor();
            }



            /**
             * Airplane
             */
            if (gamepad2.x && gamepad2.a) {
                //Release airplane servo
            }

            /**
             * Slow mode toggle
             */
            if (!gamepad1.y) {
                canToggleSlowMode = true;
            }

            if (gamepad1.y && canToggleSlowMode) {
                canToggleSlowMode = false;
                telemetry.addLine("Toggled");
                //Toggle between slow and normal speeds
                switch (rrDrive.currentSpeed) {
                    case SLOW:
                        rrDrive.toggleSlowMode(RoadRunnerDriving.Speeds.NORMAL);
                        break;
                    case NORMAL:
                        rrDrive.toggleSlowMode(RoadRunnerDriving.Speeds.SLOW);
                        break;
                }
            }


            /**
             * Telemetry data
             */
            telemetry.addData("Slide: ", arm.getSlidePosition());
            telemetry.addData("Arm pos: ", arm.getHandPosition());
            telemetry.addData("Hand power: ", arm.hand.getPower());

            telemetry.addLine();

            /*telemetry.addData("Red color front: ", colorSensors.getRedColor()[0]);
            telemetry.addData("Blue color front: ", colorSensors.getBlueColor()[0]);
            telemetry.addData("Green color front: ", colorSensors.getGreenColor()[0]);
            telemetry.addData("Red color back: ", colorSensors.getRedColor()[1]);
            telemetry.addData("Blue color back: ", colorSensors.getBlueColor()[1]);
            telemetry.addData("Green color back: ", colorSensors.getGreenColor()[1]);*/

            /*telemetry.addData("Back pixel color: ", colorSensors.getBackPixelColor());
            telemetry.addData("Front pixel color: ", colorSensors.getFrontPixelColor());*/
            /*telemetry.addData("Ratio back: ", (colorSensors.getRedColor()[1] + colorSensors.getGreenColor()[1])/colorSensors.getBlueColor()[1]);
            telemetry.addData("Ratio front: ", (colorSensors.getRedColor()[0] + colorSensors.getGreenColor()[0])/colorSensors.getBlueColor()[0]);*/


            telemetry.addData("Power: ", arm.hand.getPower());

            telemetry.update();
        }
    }
}


