package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.LedLights;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.RoadRunnerDriving;

@TeleOp(name="1 Controller Drive", group="Linear Opmode")
public class SubsystemTest extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    boolean canToggleSlowMode = true;

    boolean ALetGo = false;
    boolean Intakepos = true;
    float Default = 0f;
    float DefaultAddition = 0f;
    boolean canToggleDriverControl = true;
    double time = getRuntime();

    @Override
    public void runOpMode() {
        // create instance of lift arm class
        LiftArm arm = new LiftArm(hardwareMap);
        // create instance of intake system class
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap);
        //Instance of drive class
        RoadRunnerDriving rrDrive = new RoadRunnerDriving(hardwareMap);

        //Instance of led class
        LedLights leds = new LedLights(hardwareMap);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * Driving
             */

            if (Default == 1f){
                leds.setLed(LedLights.ledStates.RED);
            } else if (Default == 2f){
                leds.setLed(LedLights.ledStates.BLUE);
            }else if (Default == 0f){
                leds.setLed(LedLights.ledStates.DARK);
            }


            rrDrive.updateMotorsFromStick(gamepad1);
            rrDrive.update();

            telemetry.addData("Time", time);
            telemetry.addData("Tim", getRuntime());

            if (time == 10){
                leds.setLed(LedLights.ledStates.ENDGAME);
                Default = 4f;
            }

            /**
             * Intake
             */
            if (gamepad1.right_trigger > 0.75){
                intakeSystem.runIntakeSystem(1);
                leds.setLed(LedLights.ledStates.INTAKE);
                Default = Default + DefaultAddition;
                Intakepos = false;
            } else if (gamepad1.left_trigger > 0.75){
                intakeSystem.runIntakeSystem(-0.5);
                leds.setLed(LedLights.ledStates.INTAKE);
                Default = Default + DefaultAddition;
                Intakepos = false;
            } else if (!Intakepos && gamepad1.right_trigger < 0.75 && gamepad1.right_trigger < 0.75) {
                intakeSystem.runIntakeSystem(0);
                Default = Default - DefaultAddition;
            } else {
                intakeSystem.runIntakeSystem(0);
            }



            /**
             * Arm slides
             */
            if (gamepad1.dpad_up){
                //rrDrive.pixelDropRedFar();
                arm.extend(LiftArm.Distance.FULL);
            } else if (gamepad1.dpad_right){
                //rrDrive.pixelDropRedNear();
                arm.extend(LiftArm.Distance.HALF);
            } else if (gamepad1.dpad_down) {
                arm.retract();
            }

            /**
             * Trapdoor
             */
            if (gamepad1.left_bumper) {
                arm.closeTrapdoor();
                Default = Default - DefaultAddition;
            } else if (gamepad1.right_bumper) {
                arm.openTrapdoor();
                leds.setLed(LedLights.ledStates.OPEN);
                Default = Default + DefaultAddition;

            }

            /**
             * hang
             */
            if (gamepad1.a){
                arm.setArmDistance(LiftArm.Distance.ENDGAMESTART);
                ALetGo = true;
            } else if (!gamepad1.a && ALetGo) {
                arm.setArmDistance(LiftArm.Distance.ENDGAMEHOLD);
                arm.holdHang();
            }

            /**
            * plane launcher
             */
            if (gamepad1.right_stick_button){
                arm.launchPlane();
            } else if (gamepad1.left_stick_button) {
                arm.reloadPlane();
            }

            /**
             * Slow mode toggle
             */
            if (!gamepad1.y) {
                canToggleSlowMode = true;
            }

            if (gamepad1.y && canToggleSlowMode) {
                canToggleSlowMode = false;

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
            telemetry.addData("Trapdoor: ", arm.trapdoor.getPosition());
            telemetry.addData("Hand: ", arm.getHandPosition());
            telemetry.addData("Intake: ", arm.intakePower);
            telemetry.addData("Slides (R, L): ", arm.getSlidePosition() + ", " + arm.getSlidePosition());

            telemetry.addLine();

            telemetry.addData("Horizontal input", gamepad1.left_stick_x);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);

            telemetry.addData("hand power: ", arm.hand.getPower());
            telemetry.update();
        }
    }
}


