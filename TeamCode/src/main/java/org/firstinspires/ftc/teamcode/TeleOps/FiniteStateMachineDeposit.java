package org.firstinspires.ftc.teamcode.TeleOps;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Button Config for deposit
 * *X                   : high basket extend State
 * *B                   : Cancel
 * *Y                   : Specimen State
 */

public class FiniteStateMachineDeposit {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;

    //bring in the finitemachinestateintake
    private FiniteStateMachineIntake FiniteStateMachineIntake;

    /** Deposit Arm State */
    public enum LIFTSTATE {
        LIFT_START,
        HIGHBASKET_TRANSFER,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT,
        COLOR_SAMPLE_DROP,
        SPECIMEN_PICK,
        LIFT_HIGHBAR,
        LIFT_HOOK,
    }

    /** Deposit Claw State  */
    public enum DEPOSITCLAWSTATE {
        OPEN,
        CLOSE
    }

    /** ColorRange */

    class ColorRange {
        public String colorName;
        public int hueMin, hueMax;

        public ColorRange(String colorName, int hueMin, int hueMax) {
            this.colorName = colorName;
            this.hueMin = hueMin;
            this.hueMax = hueMax;
        }
    }

    /**  member declar */
    public DEPOSITCLAWSTATE depositClawState;
    
    private LIFTSTATE liftState = LIFTSTATE.LIFT_START; // Persisting state
    private ElapsedTime liftTimer = new ElapsedTime(); // Timer for controlling dumping time
 
    private ElapsedTime transfer_timer = new ElapsedTime();
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final double DEBOUNCE_THRESHOLD = 0.2; // Debouncing threshold for button presses

    List<ColorRange> colorRanges = new ArrayList<>();
    public static String detectedColor;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};
    public float hue;
    public boolean empty;

    /** constructor */
    public FiniteStateMachineDeposit(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
    }


    // Initialize Deposit Arm
    public void Init() {
        liftTimer.reset();
        robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos);
        robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(0.1);                                          // Make sure lift motor is on
        robot.liftMotorRight.setPower(0.1);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);

        /** create a list color ranges*          */
        colorRanges.add(new ColorRange("Black", 155, 170));
        colorRanges.add(new ColorRange("Red", 15, 25));
        colorRanges.add(new ColorRange("Blue", 220, 230));
        colorRanges.add(new ColorRange("Yellow", 80, 90));
    }

    // Deposit Arm Control
    public void DepositArmLoop(){
        /** determine the Color */
        Color.RGBToHSV(
                robot.colorSensor.red() * 8,
                robot.colorSensor.green() * 8,
                robot.colorSensor.blue() * 8,
                hsvValues);
        hue = hsvValues[0];

        //detect the color
        detectedColor = "None";
        empty = true;

        for(ColorRange range : colorRanges){
            if(hue > range.hueMin && hue < range.hueMax)
            {
                detectedColor = range.colorName;
                empty = false;
                break;
            }
        }

        /** FSM Loop*/
        switch (liftState) {
            case LIFT_START:
                /** DECIDE THE COLOR FOR HIGH BASKET LIFT TRANSFER - Button X - Debounce the button press X for starting the lift extend */
                if (((gamepad_1.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)||
                        (gamepad_2.getButton(GamepadKeys.Button.X)&& gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    transfer_timer.reset();
                    liftState = LIFTSTATE.HIGHBASKET_TRANSFER;
                }

                // "right trigger" and "Y" button to set deposit arm to pick specimen position
                if (((gamepad_1.getButton(GamepadKeys.Button.Y) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)||
                        (gamepad_2.getButton(GamepadKeys.Button.Y)&& gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <0.1)) &&
                        debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    liftTimer.reset();
                    liftState = LIFTSTATE.COLOR_SAMPLE_DROP;
                }
                break;

            case HIGHBASKET_TRANSFER:

                if (transfer_timer.milliseconds() > 100 && !Objects.equals(detectedColor, "Black")) {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);

                    if (transfer_timer.milliseconds() > 200) {
                        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    }

                    if (transfer_timer.milliseconds() >= 250) {
                        robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos);
                        robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos);
                        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                        robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                        liftTimer.reset();
                        liftState = LIFTSTATE.LIFT_EXTEND;
                    }
                }
                else{
                    liftState = LIFTSTATE.LIFT_START;
                }
                break;

            case LIFT_EXTEND:
                // Check if the lift has reached the high position
                if (IsLiftAtPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos) && liftTimer.seconds() > RobotActionConfig.transferTime) {
                    //move deposit arm to dump
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);
                   // Move deposit wrist servo to dump position
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);
                    liftTimer.reset();
                    liftState = LIFTSTATE.LIFT_DUMP;
                }
                break;

            case LIFT_DUMP:
                // Wait for the dump time to pass
                if (liftTimer.seconds() >= RobotActionConfig.dumpTime) {
                    depositClawState = DEPOSITCLAWSTATE.OPEN;
                }
                if (liftTimer.seconds() >= RobotActionConfig.postDumpTime) {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);// Reset servo to idle
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    liftState = LIFTSTATE.LIFT_RETRACT;
                }
                break;

            case LIFT_RETRACT:
                // Check if the lift has reached the low position
                if(Servo_AtPosition(RobotActionConfig.deposit_Claw_Open) && liftTimer.seconds()>= RobotActionConfig.retractTime) {
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos); // Start retracting the lift
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos); // Start retracting the lift
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                }
                if (IsLiftDownAtPosition(RobotActionConfig.deposit_Slide_Down_Pos)) {
                    robot.liftMotorLeft.setPower(0); // Stop the motor after reaching the low position
                    robot.liftMotorRight.setPower(0);
                    liftState = LIFTSTATE.LIFT_START;
                }
                break;

            /**  2nd branch for specimen*/
            case COLOR_SAMPLE_DROP:
                robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Pick);
                robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Pick);
                if(liftTimer.seconds()>0.25){
                    depositClawState = DEPOSITCLAWSTATE.OPEN;
                }
                liftTimer.reset();
                liftState=LIFTSTATE.SPECIMEN_PICK;
                break;

            case SPECIMEN_PICK:
                robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Pick);
                robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Pick);
                if (Objects.equals(detectedColor, "Black")){
                    depositClawState = DEPOSITCLAWSTATE.CLOSE;
                }
                if (depositClawState == DEPOSITCLAWSTATE.CLOSE){
                    liftTimer.reset();
                    liftState = LIFTSTATE.LIFT_HIGHBAR;
                }
                break;

            case LIFT_HIGHBAR:
                if(gamepad_1.getButton(GamepadKeys.Button.Y) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1&&
                        debounceTimer.seconds() > DEBOUNCE_THRESHOLD){
                    debounceTimer.reset();
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Highbar_Pos); // Start Rise to highbar position
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Highbar_Pos); // Start retracting the lift
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                    liftState = LIFTSTATE.LIFT_HOOK;
                    liftTimer.reset();
                }
                break;
            case LIFT_HOOK:
                if (IsLiftAtPosition(RobotActionConfig.deposit_Slide_Highbar_Pos)) {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                }
                if (depositClawState == DEPOSITCLAWSTATE.OPEN) {
                    // move the robot away
                    // set to deposit to transfer position
                    // set timer
                    // after timer move to retract position
                    //liftState = LIFTSTATE.LIFT_RETRACT
                }

                break;
            default:
                liftState = LIFTSTATE.LIFT_START;
                break;
        }

        // Handle lift Cancel Action if 'B' button is pressed
        if ((gamepad_1.getButton(GamepadKeys.Button.B) || gamepad_2.getButton(GamepadKeys.Button.B)) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD && liftState != LIFTSTATE.LIFT_START) {
            debounceTimer.reset();

            robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
            robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
            depositClawState = DEPOSITCLAWSTATE.OPEN;
            try {
                sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            liftState = LIFTSTATE.LIFT_RETRACT;
        }

        // Claw control - Button Back
        ClawManualControl();
        DepositClawSwitch();
    }

    // Helper method to check if the lift is within the desired position threshold
    private boolean IsLiftAtPosition(int targetPosition) {
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetPosition) < 5 && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetPosition) < 5;
    }
    private boolean IsLiftDownAtPosition(int targetPosition) {
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetPosition) < 15 && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetPosition) < 15;
    }
    private boolean Servo_AtPosition(double servoClawPosition) {
        return Math.abs(robot.depositClawServo.getPosition() - servoClawPosition) < 0.01;
    }
    LIFTSTATE State(){
        return liftState;
    }


    //Claw Control
    private void ClawManualControl(){
        if((gamepad_1.getButton(GamepadKeys.Button.A) || gamepad_2.getButton(GamepadKeys.Button.A))&& debounceTimer.seconds() > DEBOUNCE_THRESHOLD){
            debounceTimer.reset();
            ToggleDeposit();
        }
    }

    //Toggle Deposit Claw Open - Close
    private void ToggleDeposit() {
        if (depositClawState == DEPOSITCLAWSTATE.OPEN) {
            depositClawState = DEPOSITCLAWSTATE.CLOSE;
        } else {
            depositClawState = DEPOSITCLAWSTATE.OPEN;
        }
    }

    //Deposit Claw Switch
    private void DepositClawSwitch() {
        if (depositClawState != DEPOSITCLAWSTATE.OPEN) {
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        } else {
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        }
    }
    public void SetDepositClawState(DEPOSITCLAWSTATE state) {
        this.depositClawState = state;
    }
}
