package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.GamepadUtils;

public class ElbowSubsystem extends SubsystemBase {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final OpMode opMode;

    private final PIDController controller;

    private int targetPosition;

    public ElbowSubsystem(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        leftMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.LEFT_ARM_MOTOR_NAME);
        rightMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.RIGHT_ARM_MOTOR_NAME);
        leftMotor.setDirection(Constants.IntakeConstants.LEFT_ARM_MOTOR_DIRECTION);
        rightMotor.setDirection(Constants.IntakeConstants.RIGHT_ARM_MOTOR_DIRECTION);

        controller = new PIDController(Constants.IntakeConstants.ARM_JOINT_P, Constants.IntakeConstants.ARM_JOINT_I, Constants.IntakeConstants.ARM_JOINT_D);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPosition = Constants.IntakeConstants.ELBOW_STARTING_POS;

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runPIDPosition(int targetPosition) {
        this.targetPosition = targetPosition;
        if(targetPosition > leftMotor.getCurrentPosition() - 5 && targetPosition < leftMotor.getCurrentPosition() + 5) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return;
        }
        double power = controller.calculate(leftMotor.getCurrentPosition(), targetPosition);
        telemetry.addData("Power to motors:", power);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    //todo software limit switch
     /** Takes the input from a controller and converts it into a plus or minus targetposition based on the previous target position.
     * 
     * @param input -> Controller Input
     * @param rate -> Rate of change per revolution (ticks)
     * @return Position (ticks)
     */
    public int convertStickToTarget(double input, int rate) {
        input = GamepadUtils.deadzone(input, Constants.DriveConstants.DEADZONE);
        telemetry.addData("Target Position:", (int) (input * rate + targetPosition));
        return (int) (input * rate + targetPosition);
    }

    public void setTargetPosition(int targetPosition) {
        if(targetPosition > Constants.IntakeConstants.MAXIMUM_ELBOW_POS) this.targetPosition = Constants.IntakeConstants.MAXIMUM_ELBOW_POS;
        if(targetPosition < Constants.IntakeConstants.MINIMUM_ELBOW_POS) this.targetPosition = Constants.IntakeConstants.MINIMUM_ELBOW_POS;
        this.targetPosition = targetPosition;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getCurrentPosition() {
        return leftMotor.getCurrentPosition();
    }

    //Commands

    public static class RunPIDCommand extends CommandBase {
        private ElbowSubsystem elbowSubsystem;
        
        public RunPIDCommand(ElbowSubsystem elbowSubsystem) {
            this.elbowSubsystem = elbowSubsystem;
        }

        @Override
        public void execute() {
            elbowSubsystem.telemetry.addData("Elbow Target position", elbowSubsystem.getTargetPosition());
            elbowSubsystem.telemetry.addData("Elbow Current position", elbowSubsystem.getCurrentPosition());
            elbowSubsystem.runPIDPosition(elbowSubsystem.getTargetPosition());
        }
    }
}