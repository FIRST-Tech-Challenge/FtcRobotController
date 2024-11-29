package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Button Config for deposit
 * *X                   : extend
 * *B                   : Cancel
 */

public class FiniteMachineStateArm {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    
    public enum LIFTSTATE {
        LIFT_START,
        LIFT_TRANSFER,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    }

    private DEPOSITSTATE depositState;
    
    private LIFTSTATE liftState = LIFTSTATE.LIFT_START; // Persisting state
    private ElapsedTime liftTimer = new ElapsedTime(); // Timer for controlling dumping time
 
    private ElapsedTime transfer_timer = new ElapsedTime();
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final double DEBOUNCE_THRESHOLD = 0.2; // Debouncing threshold for button presses

    public FiniteMachineStateArm(RobotHardware robot, GamepadEx gamepad_1,
                                 GamepadEx gamepad_2, double DEPOSIT_ARM_IDLE, double DUMP_DEPOSIT,
                                 double DUMP_TIME, double RETRACT_TIME,
                                 double DEPOSIT_IDLE, double DEPOSIT_DUMP, double CLAW_OPEN, double CLAW_CLOSE, int LIFT_LOW, int LIFT_HIGH,
                                 double UPLIFT_POWER, double DOWNLIFT_POWER) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.DEPOSIT_ARM_IDLE = DEPOSIT_ARM_IDLE;
        this.DUMP_DEPOSIT = DUMP_DEPOSIT;
        this.DUMP_TIME = DUMP_TIME;
        this.RETRACT_TIME = RETRACT_TIME;
        this.DEPOSIT_IDLE = DEPOSIT_IDLE;
        this.DEPOSIT_DUMP = DEPOSIT_DUMP;
        this.CLAW_OPEN  = CLAW_OPEN;
        this.CLAW_CLOSE = CLAW_CLOSE;
        this.LIFT_LOW = LIFT_LOW;
        this.LIFT_HIGH = LIFT_HIGH;
        this.UPLIFT_POWER = UPLIFT_POWER;
        this.DOWNLIFT_POWER = DOWNLIFT_POWER;
    }

    final double DEPOSIT_ARM_IDLE;     // Idle position for the deposit arm servo
    final double DUMP_DEPOSIT;  // Dumping position for the deposit arm servo
    final double DUMP_TIME;     // Time for dumping action in seconds
    final int LIFT_LOW;         // Encoder position for the low position
    final int LIFT_HIGH;        // Encoder position for the high position
    final double UPLIFT_POWER;  // uplife power
    final double DOWNLIFT_POWER;// downwards power
    final double DEPOSIT_IDLE;   // deposit idling position
    final double DEPOSIT_DUMP;   // deposit dump position
    final double RETRACT_TIME;  // retract waiting time
    final double CLAW_OPEN;     // claw open
    final double CLAW_CLOSE;    // claw close

    // Initialize Deposit Arm
    public void Init() {
        liftTimer.reset();
        robot.liftMotorLeft.setTargetPosition(LIFT_LOW);
        robot.liftMotorRight.setTargetPosition(LIFT_LOW);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(0.1);                                          // Make sure lift motor is on
        robot.liftMotorRight.setPower(0.1);
        robot.depositWristServo.setPosition(DEPOSIT_IDLE);
        robot.depositLeftArmServo.setPosition(DEPOSIT_ARM_IDLE);
        robot.depositRightArmServo.setPosition(DEPOSIT_ARM_IDLE);
        robot.depositClawServo.setPosition(CLAW_OPEN);
    }

    // Deposit Arm Control
    public void DepositArmLoop() {
        // Display current lift state and telemetry feedback
        switch (liftState) {
            case LIFT_START:
                // Debounce the button press for starting the lift extend
                if ((gamepad_1.getButton(GamepadKeys.Button.X) || gamepad_2.getButton(GamepadKeys.Button.X)) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    transfer_timer.reset();
                    liftState = LIFTSTATE.LIFT_TRANSFER;
                }
            case LIFT_TRANSFER:
                if (transfer_timer.seconds() >= 0.15) {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                }
                if (transfer_timer.seconds() >= 0.25) {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                }
                if (transfer_timer.seconds() >= 0.35) {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
                }
                if (transfer_timer.seconds() >= 0.85) {
                    robot.liftMotorLeft.setTargetPosition(LIFT_HIGH);
                    robot.liftMotorRight.setTargetPosition(LIFT_HIGH);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(UPLIFT_POWER);
                    robot.liftMotorRight.setPower(UPLIFT_POWER);
                    liftState = LIFTSTATE.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                // Check if the lift has reached the high position
                if (isLiftAtPosition(LIFT_HIGH)) {
                    //move deposit arm to dump
                    robot.depositLeftArmServo.setPosition(DUMP_DEPOSIT);
                    robot.depositRightArmServo.setPosition(DUMP_DEPOSIT);
                   // Move deposit wrist servo to dump position
                    robot.depositWristServo.setPosition(DEPOSIT_DUMP);
                    liftTimer.reset();
                    liftState = LIFTSTATE.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                // Wait for the dump time to pass
                if (liftTimer.seconds() >= DUMP_TIME) {
                    robot.depositClawServo.setPosition(CLAW_OPEN);
                    depositState = DEPOSITSTATE.OPEN;
                }
                if (liftTimer.seconds() >= DUMP_TIME+0.5) {
                    robot.depositLeftArmServo.setPosition(DEPOSIT_ARM_IDLE);// Reset servo to idle
                    robot.depositRightArmServo.setPosition(DEPOSIT_ARM_IDLE);
                    robot.depositWristServo.setPosition(DEPOSIT_IDLE);
                    liftState = LIFTSTATE.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                // Check if the lift has reached the low position
                if(servo_AtPosition(CLAW_OPEN) && liftTimer.seconds()>= RETRACT_TIME) {
                    robot.liftMotorLeft.setTargetPosition(LIFT_LOW); // Start retracting the lift
                    robot.liftMotorRight.setTargetPosition(LIFT_LOW); // Start retracting the lift
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(DOWNLIFT_POWER);
                    robot.liftMotorRight.setPower(DOWNLIFT_POWER);
                }
                if (isLiftAtPosition(LIFT_LOW)) {
                    robot.liftMotorLeft.setPower(0); // Stop the motor after reaching the low position
                    robot.liftMotorRight.setPower(0);
                    liftState = LIFTSTATE.LIFT_START;
                }
                break;
            default:
                liftState = LIFTSTATE.LIFT_START;
                break;
        }

        // Handle lift Cancel Action if 'B' button is pressed
        if ((gamepad_1.getButton(GamepadKeys.Button.B) || gamepad_2.getButton(GamepadKeys.Button.B)) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD && liftState != LIFTSTATE.LIFT_START) {
            debounceTimer.reset();
            liftState = LIFTSTATE.LIFT_START;
            robot.liftMotorLeft.setPower(0); // Ensure the motor is stopped
            robot.liftMotorRight.setPower(0);
            robot.depositWristServo.setPosition(DEPOSIT_IDLE);
            robot.depositLeftArmServo.setPosition(DEPOSIT_ARM_IDLE);
            robot.depositRightArmServo.setPosition(DEPOSIT_ARM_IDLE);
            robot.depositClawServo.setPosition(CLAW_OPEN);
        }

        // Claw control - Button Back
        if((gamepad_1.getButton(GamepadKeys.Button.Y) || gamepad_2.getButton(GamepadKeys.Button.Y))&& debounceTimer.seconds() > DEBOUNCE_THRESHOLD){
            debounceTimer.reset();
            ToggleDeposit();
            if (depositState == DEPOSITSTATE.OPEN) {
                robot.depositClawServo.setPosition(CLAW_CLOSE);
            } else {
                robot.depositClawServo.setPosition(CLAW_OPEN);
            }
        }
    }

    // Helper method to check if the lift is within the desired position threshold
    private boolean isLiftAtPosition(int targetPosition) {
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetPosition) < 5 && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetPosition) < 5;
    }

    private boolean servo_AtPosition(double servoClawPosition) {
        return Math.abs(robot.depositClawServo.getPosition() - servoClawPosition) < 0.01;
    }
    LIFTSTATE State(){
        return liftState;
    }

    //Deposit Claw State
    public enum DEPOSITSTATE {
        OPEN,
        CLOSE
    }

    //Toggle Deposit Claw Open - Close
    private void ToggleDeposit() {
        if (depositState == DEPOSITSTATE.OPEN) {
            depositState = DEPOSITSTATE.CLOSE;
        } else {
            depositState = DEPOSITSTATE.OPEN;
        }
    }
}
