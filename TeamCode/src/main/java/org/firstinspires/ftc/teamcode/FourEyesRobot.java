package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class FourEyesRobot extends Mecanum {
    HardwareMap hardwareMap;
    Lift lift = new Lift(hardwareMap);
    Claw claw = new Claw(hardwareMap);
    Arm arm = new Arm(hardwareMap);
    ActiveIntake activeIntake = new ActiveIntake(hardwareMap);

    public FourEyesRobot(HardwareMap hw) {
        super(hw);
        hardwareMap = hw;
    }
    public void openClaw() {
        claw.openClaw();
    }
    public void closeClaw() {
        claw.closeClaw();
    }
    public void moveLift(double power) {
        lift.moveLift(power);
    }
    public void intakeForward() {
        activeIntake.activateIntake();
    }
    public void intakeBackward() {
        activeIntake.reverseIntake();
    }
    public void intakeStop() {
        activeIntake.deactivateIntake();
    }
    public boolean isIntakeing() {
        return activeIntake.isRunning();
    }
    public void changeHeightArm(double height) {
        arm.changeHeight(height);
    }

}
