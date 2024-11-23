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

    private final GamepadEx gamepad;
    private final RobotHardware robot;

    //declear deposit servo position
    public static double depositLeftArm = 0.5;
    public static double depositRighttArm = 0.5;
    private double servoposition;
    int iniPosition = 50;
    int delta_Position = 50;
    int current_Position;
    private static final double speed = 0.4;

   //
    private final ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private static final double DEBOUNCE_THRESHOLD = 0.25;

    public ServoTest(RobotHardware robot, GamepadEx gamepad) {
        this.robot = robot;
        this.gamepad= gamepad;
    }

    public void ServoTestInit(){
        robot.depositLeftArmServo.setPosition(0.0);
        robot.depositRightArmServo.setPosition(0.0);
        robot.depositWristServo.setPosition(0.0);
        robot.depositClawServo.setPosition(0.0);

        robot.intakeLeftArmServo.setPosition(0.7);
        robot.intakeRightArmServo.setPosition(0.7);
        robot.intakeSlideServo.setPosition(0.4);

        depositLeftArm = robot.depositLeftArmServo.getPosition();
        depositRighttArm = robot.depositRightArmServo.getPosition();

        robot.liftMotorLeft.setTargetPosition(iniPosition);
        robot.liftMotorRight.setTargetPosition(iniPosition);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
    }

    public void ServoTestLoop() {
        if (gamepad.getButton(A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();

            //use to be 0.01

            robot.depositClawServo.setPosition(Range.clip(0,0,1));
        }
        if (gamepad.getButton(B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            //use to be 0.01

            robot.depositClawServo.setPosition(Range.clip(0.11,0,1));
        }

        if (gamepad.getButton(DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositRightArmServo.getPosition();
            //use to be 0.01
            servoposition += 0.05;
            robot.depositLeftArmServo.setPosition(Range.clip(servoposition,0,1));
            robot.depositRightArmServo.setPosition(Range.clip(servoposition,0,1));

        }

        if (gamepad.getButton(DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositLeftArmServo.getPosition();
            //use to be 0.01
            servoposition -= 0.05;
            robot.depositLeftArmServo.setPosition(Range.clip(servoposition,0,1));
            robot.depositRightArmServo.setPosition(Range.clip(servoposition,0,1));

        }

        if (gamepad.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition();
            //use to be 0.01
            servoposition += 0.05;
            robot.depositWristServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad.getButton(DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition();
            //use to be 0.01
            servoposition -= 0.05;
            robot.depositWristServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad.getButton(X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {

            current_Position = robot.liftMotorLeft.getCurrentPosition();

            robot.liftMotorLeft.setTargetPosition(Range.clip(current_Position + delta_Position,50,3000));
            robot.liftMotorRight.setTargetPosition(Range.clip(current_Position + delta_Position,50,3000));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }
        if (gamepad.getButton(Y) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {

            current_Position = robot.liftMotorLeft.getCurrentPosition();

            robot.liftMotorLeft.setTargetPosition(Range.clip(current_Position - delta_Position,50,3000));
            robot.liftMotorRight.setTargetPosition(Range.clip(current_Position - delta_Position,50,3000));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }
    }

}


