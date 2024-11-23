package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.GamepadUtils;

import org.firstinspires.ftc.teamcode.Constants;

public class LinearSlideSubsystem extends SubsystemBase {
    private final DcMotorEx motor;

    private final HardwareMap hardwareMap;
    private final OpMode opMode;
    private final Telemetry telemetry;

    private final PIDController controller;

    private int targetPosition;

    public LinearSlideSubsystem(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        motor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_NAME);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_DIRECTION);

        targetPosition = Constants.IntakeConstants.LINEAR_STARTING_POS;

        controller = new PIDController(Constants.IntakeConstants.LINEAR_SLIDE_P, Constants.IntakeConstants.LINEAR_SLIDE_I, Constants.IntakeConstants.LINEAR_SLIDE_D);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runPIDPosition(int targetPosition) {
        this.targetPosition = targetPosition;
        if(motor.getCurrentPosition() > targetPosition - 5 && motor.getCurrentPosition() < targetPosition + 5) {
            motor.setPower(0);
            return;
        }
        telemetry.addData("Power to motors:", controller.calculate(motor.getCurrentPosition(), targetPosition));
        motor.setPower(controller.calculate(motor.getCurrentPosition(), targetPosition));
    }

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

    public int getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(int targetPosition) {
        if(targetPosition > Constants.IntakeConstants.MAXIMUM_SLIDE_POS) this.targetPosition = Constants.IntakeConstants.MAXIMUM_SLIDE_POS;
        if(targetPosition < Constants.IntakeConstants.MINIMUM_SLIDE_POS) this.targetPosition = Constants.IntakeConstants.MINIMUM_SLIDE_POS;
        this.targetPosition = targetPosition;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    //Commands

    public static class RunPIDCommand extends CommandBase {
        private LinearSlideSubsystem linearSlideSubsystem;
        
        public RunPIDCommand(LinearSlideSubsystem linearSlideSubsystem) {
            this.linearSlideSubsystem = linearSlideSubsystem;
        }

        @Override
        public void execute() {
            linearSlideSubsystem.telemetry.addData("Target position", linearSlideSubsystem.getTargetPosition());
            linearSlideSubsystem.telemetry.addData("Current position", linearSlideSubsystem.getCurrentPosition());
            linearSlideSubsystem.runPIDPosition(linearSlideSubsystem.getTargetPosition());
        }
    }
}