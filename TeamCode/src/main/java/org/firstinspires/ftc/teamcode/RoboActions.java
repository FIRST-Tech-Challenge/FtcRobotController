package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Claw;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Hang;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Wrist;
import org.firstinspires.ftc.teamcode.Usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.Usefuls.Math.M;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

public class RoboActions {
    public Arm arm;
    public Slides slides;
    Drivetrain drive;
    public Claw claw;
    public Wrist wrist;
    Hang hang;

    public RoboActions(HardwareMap hardwareMap, Pose2d startPosition){
        drive = new Drivetrain(hardwareMap, startPosition);
        slides = new Slides(hardwareMap, drive.getSlidesMotor());
        arm = new Arm(hardwareMap, drive.getArmMotor());
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap);
        hang = new Hang(hardwareMap);
    }
    public void slidesUpdate() {
        slides.update();
    }
    public void armUpdate() {
        arm.update();
    }
    public void preScore() {
        arm.deposit();
        slides.preScore();
    }
    public void drivePowers(double y, double x, double r) {
        drive.setPowers(y, x, r);
    }
    // Teleop
    public void tDefault() {
        arm.preTake();
        slides.floorIntake();
        wrist.intake();
        claw.open();
    }
    public void intakeReady(){
        arm.intake();
    }
    public void closeIntake() {
        claw.close();
    }
    public void finishedIntake () {
        arm.preTake();
        slides.floorIntake();
    }
    public void readyDeposit() {
        preScore();
        wrist.intake();
    }
    public void slidesBeginScore() {
        slides.score();
    }
    public void wristScore() {
        wrist.deposit();
    }
    public void finishScore() {
        claw.open();
    }
    public void postScore() {
        wrist.intake();
        slides.preScore();
    }
    public void beginSubmersible() {
        wrist.intake();
        arm.preSubmerse();
        claw.open();
    }
    public void submersibleSlider(double slideInches) {
        wrist.intake();
        arm.preSubmerse();
        claw.open();
        slides.setTargetSlidesPosition(slideInches);
    }
    public void submersibleIntakeOpen() {
        claw.open();
        arm.intake();
        wrist.intake();
    }
    public void submersibleFinish1() {
        wrist.deposit();
        arm.preSubmerse();
    }
    public void specimenIntake() {
        wrist.specimen();
        claw.open();
        arm.preTake();
        slides.setTargetSlidesPosition(1.5);
    }
    public void finishSpecimenIntake() {
        claw.close();
    }
    public void dropSpecimenIntake() {
        arm.specimenIntake();
    }
    public void holdingSpecimen() {
        slides.setTargetSlidesPosition(0);
        wrist.holdSpecimen();
        arm.holdSpecimen();
    }
    public void returnToMain() {
        wrist.intake();
    }
    public void hangPower (double power) {
        hang.move(power);
    }
}
