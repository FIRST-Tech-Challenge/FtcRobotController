package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorPair;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ElbowSubsystem extends SubsystemBase {
    private DcMotorPair elbowMotors;
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    
    private HardwareMap hardwareMap;

    private PIDController controller;

    private int targetPosition;

    public ElbowSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        leftMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.LEFT_ARM_MOTOR_NAME);
        rightMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.RIGHT_ARM_MOTOR_NAME);
        leftMotor.setDirection(Constants.IntakeConstants.LEFT_ARM_MOTOR_DIRECTION);
        rightMotor.setDirection(Constants.IntakeConstants.RIGHT_ARM_MOTOR_DIRECTION);

        controller = new PIDController(Constants.IntakeSubsystem.ARM_JOINT_P, Constants.IntakeConstants.ARM_JOINT_I, Constants.IntakeConstants.ARM_JOINT_D);

        elbowMotors = new DcMotorPair(leftMotor, rightMotor);
        elbowMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_REST_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runPIDPosition(int targetPosition) {
        this.targetPosition = targetPosition;
        if(targetPosition > leftMotor.getCurrentPosition() - 5 || targetPosition < leftMotor.getCurrentPosition() + 5) {
            elbowMotors.setPower(0);
            return;
        }
        elbowMotors.setPower(controller.calculate(leftMotor.getCurrentPosition(), targetPosition));
    }

    public boolean hasReachedTarget() {
        if(targetPosition > leftMotor.getCurrentPosition() - 5 || targetPosition < leftMotor.getCurrentPosition() + 5) return true;
        else return false;
    }

    public void stopMotors() {
        elbowMotors.setPower(0);
    }

     /** Takes the input from a controller and converts it into a plus or minus targetposition based on the previous target position.
     * 
     * @param input -> Controller Input
     * @param rate -> Rate of change per revolution (ticks)
     * @return Position (ticks)
     */
    public int convertStickToTarget(double input, int rate) {
        input = GamepadUtils.deadzone(input, Constants.DriveConstants.DEADZONE);
        return (int) (input * rate + targetPosition);
    }

    //Commands

    public static class RunPIDCommand extends CommandBase {
        private int targetPosition;
        private ElbowSubsystem elbowSubsystem;
        
        public RunPIDCommand(int targetPosition, ElbowSubsystem elbowSubsystem) {
            this.targetPosition = targetPosition;
            this.elbowSubsystem = elbowSubsystem;
        }

        @Override
        public void execute() {
            elbowSubsystem.runPIDPosition(targetPosition);
        }

        @Override
        public boolean isFinished() {
            return elbowSubsystem.hasReachedTarget();
        }

        @Override
        public void end() {
            elbowSubsystem.stopMotors();
        }
    }
}