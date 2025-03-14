package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmControl extends SubsystemBase {
    private final DcMotorEx slide;
    private final DcMotorEx arm;

    public ArmControl(HardwareMap hardwareMap) {
        slide = hardwareMap.get(DcMotorEx.class, "Slide");
        arm = hardwareMap.get(DcMotorEx.class, "Arm");

        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // ✅ Generalized method to move both the Arm and Slide
    public void moveArmAndSlide(int armPos, int slidePos, double armVelocity, double slideVelocity) {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> setMotorPositionAndVelocity(arm, armPos, armVelocity)),
                        new InstantCommand(() -> setMotorPositionAndVelocity(slide, slidePos, slideVelocity))
                )
        );
    }

    // ✅ Helper function to set position and velocity for any motor
    private void setMotorPositionAndVelocity(DcMotorEx motor, int position, double velocity) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setVelocity(velocity);
    }

    public boolean isBusy(){
        return arm.isBusy();
    }
    public boolean isSlideBusy(){
        return slide.isBusy();
    }
}