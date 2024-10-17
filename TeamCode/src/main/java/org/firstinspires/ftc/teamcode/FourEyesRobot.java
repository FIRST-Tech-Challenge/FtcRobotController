package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class FourEyesRobot extends Mecanum {
    HardwareMap hardwareMap;
    Lift lift;

    Arm arm;

    Wrist wrist;
    Claw claw;
    ActiveIntake activeIntake;

    public FourEyesRobot(HardwareMap hw) {
        super(hw);
        //Reassigned here to ensure that they are properly initialized
        hardwareMap = hw;
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
        activeIntake = new ActiveIntake(hardwareMap);
    }
    public void toggleClaw() {
        claw.toggleClaw();
    }

    public void moveLift(double power) {
        lift.moveLift(power);
    }
    public void intakeForward() {
        wrist.setIntakeMode();
        activeIntake.activateIntake();
    }
    public void intakeBackward() {
        activeIntake.reverseIntake();
    }
    public void intakeStop() {
        wrist.setHoverMode();
        activeIntake.deactivateIntake();
    }
    public void depositBasket(){
        wrist.setDepositMode();
    }
    public boolean isIntaking() {
        return activeIntake.isRunning();
    }
    public void changeHeightArm(double height) {
        arm.changeHeight(height);
    }

}
