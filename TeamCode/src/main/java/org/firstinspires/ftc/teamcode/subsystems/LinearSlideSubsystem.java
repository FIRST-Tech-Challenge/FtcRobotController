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
import org.firstinspires.ftc.teamcode.utils.GamepadUtils;

import org.firstinspires.ftc.teamcode.Constants;

public class LinearSlideSubsystem extends SubsystemBase {
    private DcMotorEx motor;

    private HardwareMap hardwareMap;

    private PIDController controller;

    private int targetPosition;

    public LinearSlideSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        motor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_NAME);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_DIRECTION);

        controller = new PIDController(Constants.IntakeConstants.LINEAR_SLIDE_P, Constants.IntakeConstants.LINEAR_SLIDE_I, Constants.IntakeConstants.LINEAR_SLIDE_D);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopMotor() {
        motor.setPower(0);
    }

    public boolean hasReachedTarget() {
        if(motor.getCurrentPosition() > targetPosition - 5 || motor.getCurrentPosition() < targetPosition + 5) return true;
        else return false; 
    }

    public void runPIDPosition(int targetPosition) {
        this.targetPosition = targetPosition;
        if(motor.getCurrentPosition() > targetPosition - 5 || motor.getCurrentPosition() < targetPosition + 5) {
            motor.setPower(0);
            return;
        }
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
        return (int) (input * rate + targetPosition);
    }

    //Commands

    public static class RunPIDCommand extends CommandBase {
        private int targetPosition;
        private LinearSlideSubsystem linearSlideSubsystem;
        
        public RunPIDCommand(int targetPosition, LinearSlideSubsystem linearSlideSubsystem) {
            this.targetPosition = targetPosition;
            this.linearSlideSubsystem = linearSlideSubsystem;
        }

        @Override
        public void execute() {
            linearSlideSubsystem.runPIDPosition(targetPosition);
        }

        @Override
        public boolean isFinished() {
            return linearSlideSubsystem.hasReachedTarget();
        }

        @Override
        public void end() {
            linearSlideSubsystem.stopMotor();
        }
    }
}