package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopTest extends LinearOpMode {
    //private final double robot_power = 1.0;
    BBRobot robot;
    enum DriveState {
        MOVING,
        ARM_CONTROL
    };
    private DriveState currentState = DriveState.MOVING;

    @Override
    public void runOpMode() {

        robot = new BBRobot(hardwareMap, telemetry);
        robot.isTeleOp = true;

        telemetry.addData("Status", "Initialized"); //Displays "Status: Initialized"
        telemetry.update();

        //Driver must press INIT and then ▶️
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("TeleOP", "New Code");
            telemetry.update();
            switch(currentState) {
                case MOVING:
                    handleMovement();
                    if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 &&
                        gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0)
                    {
                        currentState = DriveState.ARM_CONTROL;
                    }
                    break;
                case ARM_CONTROL:
                    handleControl();
                    if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 ||
                            gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0)
                    {
                        currentState = DriveState.MOVING;
                    }
                    break;
            }
        }
    }

    private void handleMovement() {
        /********** GamePad 1 ****************/
        //Turning
        if (this.gamepad1.left_stick_x < 0.5 && this.gamepad1.left_stick_x > -0.5) {
            robot.turnOff();
        }
        if (this.gamepad1.right_stick_x > 0.5) {
            robot.turnForTime(robot.robot_power, 10, false, -1);
        }
        if (this.gamepad1.right_stick_x < -0.5) {
            robot.turnForTime(robot.robot_power, 10, false, 1);
        }

        // Moving
        if (this.gamepad1.right_stick_x < 0.5 && this.gamepad1.right_stick_x > -0.5 && this.gamepad1.right_stick_y < 0.5 && this.gamepad1.right_stick_y > -0.5) {
            robot.turnOff();
        }
        if (this.gamepad1.left_stick_y > 0.5) {
            robot.moveForward(robot.robot_power);
        }
        if (this.gamepad1.left_stick_y < -0.5) {
            robot.moveBackward(robot.robot_power);
        }
        if (this.gamepad1.left_stick_x > 0.5) {
            robot.moveRight(robot.robot_power);
        }
        if (this.gamepad1.left_stick_x < -0.5) {
            robot.moveLeft(robot.robot_power);
        }
    }

    private void handleControl() {

        /**** DRIVER 1 *****/

        //pick using active intake
        if (this.gamepad1.dpad_up == true) {
            robot.pickIntake();
        }
        // align to the platform
        if (this.gamepad1.dpad_down == true) {
            robot.wrist_end();
        }
        // dopr the wrong element
        if (this.gamepad1.dpad_left == true) {
            robot.wrist_mid();
            robot.dropIntake();
        }
        //active intake at driving level
        if (this.gamepad1.dpad_right == true) {
            robot.wrist_mid();
        }

        // drop in the bucket
        if (this.gamepad1.b == true) {
            robot.dropElement();
        }

        // hang on bar
        if (this.gamepad1.right_trigger > 0.5) {
            robot.moveBackwardToPosition(robot.robot_power,5, 1500);
            robot.hangElementOnHighBar(robot.robot_power);
        }
//        if (this.gamepad1.left_trigger > 0.5) {
//                robot.armPark();
//        }

        if (this.gamepad1.dpad_up == false && this.gamepad1.dpad_down == false) {
            robot.turnOff();
        }

        // hang on bar
        if (this.gamepad1.left_bumper == true) {
            robot.trayOpen();
        }
        if (this.gamepad1.right_bumper == true) {
            robot.trayClose();
        }

        /********** DRIVER 2 ****************/
        // Code functions for gamepad 2

        // Use gamepad buttons to move the slide
        // expand
        if (this.gamepad2.dpad_up) {
            robot.expandSlide();
        }
        // contract
        if (this.gamepad2.dpad_down) {
            robot.contractSlide();
        }
        //slow contract
        if (this.gamepad2.dpad_left == true) {
            robot.contractSlideSlowRealtively(5);
        }
        //slow expand
        if (this.gamepad2.dpad_right == true) {
            robot.expandSlideSlowRealtively(5);
        }

        // hang on bar
        if (this.gamepad2.left_trigger > 0.5) {
            robot.pickElementForBar(robot.robot_power);
        }

        // transfer pixel from intake to claw
        if (this.gamepad2.a == true) {
            robot.elementGrab();
        }

        if (this.gamepad2.x == true) {
            robot.clawOpen();
        }
        if (this.gamepad2.y == true) {
            robot.clawClose();
        }
        // hang on bar
        if (this.gamepad2.left_bumper == true) {
            robot.expandLA();
        }
        if (this.gamepad2.right_bumper == true) {
            robot.contractLA();
        }

        if (this.gamepad2.left_stick_y > 0.5) {
            robot.slideForwardIntake();
        }
        if (this.gamepad2.left_stick_y < -0.5) {
            robot.slideBackIntake();
        }
        if (this.gamepad2.left_stick_y > 0.5) {
            robot.slideIntakeReset();
        }

        if (this.gamepad2.right_stick_y > 0.5) {
            robot.arm_pick();
        }
        if (this.gamepad2.right_stick_y < -0.5) {
            robot.arm_end();
        }
        if (this.gamepad2.right_stick_x > 0.5) {
            robot.arm_straight();
        }
        if (this.gamepad2.right_stick_x < -0.5) {
            robot.arm_drop();
        }

    };
}