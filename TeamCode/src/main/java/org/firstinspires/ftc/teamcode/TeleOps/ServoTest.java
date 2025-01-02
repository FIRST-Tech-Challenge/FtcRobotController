package org.firstinspires.ftc.teamcode.TeleOps;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ServoTest{

    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;

    //declear initial servo position
    private double servoposition;
    int iniPosition = 50;
    int delta_Position = 50;
    int current_Position;
    private static final double speed = 0.4;

   //
    private final ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private static final double DEBOUNCE_THRESHOLD = 0.25;

    public ServoTest(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
    }

    public void ServoTestInit(){
        robot.depositArmServo.setPosition(0.0);
        robot.depositWristServo.setPosition(0.0);
        robot.depositClawServo.setPosition(0.0);

        robot.intakeLeftArmServo.setPosition(0.0);
        robot.intakeRightArmServo.setPosition(0.0);
        robot.intakeLeftSlideServo.setPosition(0.4);
        robot.intakeRightSlideServo.setPosition(0.4);
        robot.intakeWristServo.setPosition(0.0);
        robot.intakeRotationServo.setPosition(0.46);
        robot.intakeClawServo.setPosition(0.5);

        robot.liftMotorLeft.setTargetPosition(iniPosition);
        robot.liftMotorRight.setTargetPosition(iniPosition);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
    }

    public void ServoTestLoop() {
        /*deposit system setup*/
        // A for claw open
        if (gamepad_1.getButton(A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();

            //use to be 0.01
            servoposition = robot.depositArmServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.depositClawServo.setPosition(Range.clip(servoposition,0,1));
        }
        //B for claw close
        if (gamepad_1.getButton(B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            //use to be 0.01
            servoposition = robot.depositArmServo.getPosition();
            servoposition -= 0.01;
            robot.depositClawServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositArmServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.depositArmServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositArmServo.getPosition();
            //use to be 0.01
            servoposition -= 0.01;
            robot.depositArmServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.depositWristServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition();
            //use to be 0.01
            servoposition -= 0.01;
            robot.depositWristServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {

            current_Position = robot.liftMotorLeft.getCurrentPosition();

            robot.liftMotorLeft.setTargetPosition(Range.clip(current_Position + delta_Position,50,3500));
            robot.liftMotorRight.setTargetPosition(Range.clip(current_Position + delta_Position,50,3500));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }
        if (gamepad_1.getButton(Y) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {

            current_Position = robot.liftMotorLeft.getCurrentPosition();

            robot.liftMotorLeft.setTargetPosition(Range.clip(current_Position - delta_Position,50,3500));
            robot.liftMotorRight.setTargetPosition(Range.clip(current_Position - delta_Position,50,3500));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }

        /* Intake System */
        // gamepad2 DPAD_UP button for arm rise
        if (gamepad_2.getButton(DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();

            //use to be 0.01
            servoposition = robot.intakeLeftArmServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.intakeLeftArmServo.setPosition(Range.clip(servoposition,0,1));
            robot.intakeRightArmServo.setPosition(Range.clip(servoposition,0,1));
        }
        // gamepad2 DPAD_DOWN for arm lower
        if (gamepad_2.getButton(DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            //use to be 0.01
            servoposition = robot.intakeLeftArmServo.getPosition();
            servoposition -= 0.01;
            robot.intakeLeftArmServo.setPosition(Range.clip(servoposition,0,1));
            robot.intakeRightArmServo.setPosition(Range.clip(servoposition,0,1));
        }
        // gamepad2 A button for claw open
        if (gamepad_2.getButton(A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeClawServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.intakeClawServo.setPosition(Range.clip(servoposition,0,1));
        }
        // gamepad2 B buttion for claw close
        if (gamepad_2.getButton(B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeClawServo.getPosition();
            //use to be 0.01
            servoposition -= 0.01;
            robot.intakeClawServo.setPosition(Range.clip(servoposition,0,1));
        }

        // gamepad2 DPAD_LEFT for intake wrist servo
        if (gamepad_2.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.intakeWristServo.setPosition(Range.clip(servoposition,0,1));
        }

        // gamepad2 DPAD_RIGHT for intake wrist servo
        if (gamepad_2.getButton(DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition();
            //use to be 0.01
            servoposition -= 0.01;
            robot.intakeWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        //gamepad2 X for intake claw rotation
        // gamepad2 DPAD_LEFT for intake wrist servo
        if (gamepad_2.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.intakeWristServo.setPosition(Range.clip(servoposition,0,1));
        }

        // gamepad2 DPAD_RIGHT for intake wrist servo
        if (gamepad_2.getButton(X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeRotationServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.intakeRotationServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        // gamepad2 DPAD_RIGHT for intake wrist servo
        if (gamepad_2.getButton(Y) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeRotationServo.getPosition();
            //use to be 0.01
            servoposition -= 0.01;
            robot.intakeRotationServo.setPosition(Range.clip(servoposition, 0, 1));
        }
    }

}


