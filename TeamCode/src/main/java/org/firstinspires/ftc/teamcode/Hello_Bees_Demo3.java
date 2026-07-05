package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.susbsystems.robot_system;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name = "!Hello Bees")
public class Hello_Bees_Demo3 extends OpMode {
    robot_system robot;
    ButtonBlock stopArm, startFogCycle, stopFogCycle, startTreatment;
    ButtonBlock stopShoulder,shoulderHome;
    ButtonBlock stopHomeShoulder, homeArm, startStopFogCycle;
    ButtonBlock stopAll,toggle_arm_full;
    ButtonBlock lockTarget, unlockTarget;
    ButtonBlock dpadUp, dpadDown, dpadLeft, dpadRight;
    ButtonBlock leftbumper, rightbumper;
    boolean arm_full_toggle = true;
    int yPosTarget = 0;
    private final Position ikTestTargetA = new Position(DistanceUnit.INCH, -32, 0, 10, 0);
    private final Position ikTestTargetB = new Position(DistanceUnit.INCH, -36, 4, 14, 0);
    private final Position ikTestTargetC = new Position(DistanceUnit.INCH, -28, -3, 8, 0);
    private final Position ikTestTargetD = new Position(DistanceUnit.INCH, -40, 7, 18, 0);

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new robot_system(hardwareMap);
        toggle_arm_full = new ButtonBlock().onTrue(() -> {arm_full_toggle = !arm_full_toggle;});
        stopAll = new ButtonBlock()
                .onTrue(() -> {
                    robot.stopArmToPosition();
                    robot.startFogCycle();
                });
        stopArm = new ButtonBlock()
                .onTrue(() -> {robot.stopArmToPosition();});

        startFogCycle = new ButtonBlock()
                .onTrue(() -> {robot.startFogCycle();});
        stopFogCycle = new ButtonBlock()
                .onTrue(() -> {robot.stopFogCycle();});
        stopShoulder = new ButtonBlock()
                .onTrue(() -> {robot.shoulderStop();});
        shoulderHome = new ButtonBlock()
                .onTrue(() -> {robot.shoulderHome();});
        stopHomeShoulder = new ButtonBlock()
                .onTrue(()-> {
                    if (robot.shoulderIsBusy()) {robot.shoulderStop();}
                    else {robot.shoulderHome();}});
        startStopFogCycle = new ButtonBlock()
                .onTrue(()-> {
                    if (robot.isCycling()) {robot.stopFogCycle();}
                    else {robot.startFogCycle();}});
        homeArm = new ButtonBlock()
                .onTrue(()-> {
                    robot.armHome();});
        startTreatment = new ButtonBlock()
                .onTrue(()->{robot.startTreatment(arm_full_toggle);});
        lockTarget = new ButtonBlock()
                .onTrue(() -> {robot.lockCurrentFilteredTarget(true);});
        unlockTarget = new ButtonBlock()
                .onTrue(() -> {
                    robot.testStopAutoMove();
                    robot.unlockTreatmentTarget();
                });
        dpadUp = new ButtonBlock().onTrue(() -> {robot.setCyclecount((robot.getCycleTarget()+1));});
        dpadDown = new ButtonBlock().onTrue(() -> {robot.setCyclecount((robot.getCycleTarget()-1));});
        dpadLeft = new ButtonBlock().onTrue(() -> {robot.setFogTime((robot.getFogTime()+1));});
        dpadRight = new ButtonBlock().onTrue(() -> {robot.setFogTime((robot.getFogTime()-1));});
        leftbumper = new ButtonBlock().onTrue(() -> {robot.setFanTime((robot.getFanTime()+1));});
        rightbumper = new ButtonBlock().onTrue(() -> {robot.setFanTime((robot.getFanTime()-1));});
        robot.wristGoPos(.9);
        telemetry.addLine("Initialized");
        telemetry.update();

    }
    @Override
    public void loop() {
        buttonEvents();
        robot.robot_drive(gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (!robot.shoulderIsBusy()) robot.shoulderSetPower(-gamepad2.right_stick_y);
        else if (Math.abs(gamepad2.right_stick_y) > 0) {
            robot.shoulderStop();
            robot.shoulderSetPower(-gamepad2.right_stick_y);
        }
        if(gamepad2.dpad_up){robot.moveWristManual(.5);}
        if(gamepad2.dpad_down){robot.moveWristManual(-.5);}
        robot.moveExtensionManual(gamepad2.left_stick_y);
        robot.moveTurretManual(gamepad2.left_trigger - gamepad2.right_trigger);
        robot.update();

        telemetry();
    }
    private void buttonEvents() {
        // Simplified down all into classes that handle button blocks the same way that a normal one does

        //startStopFogCycle.update(gamepad1.b);
        stopAll.update(gamepad1.a);
        shoulderHome.update(gamepad2.b);
        homeArm.update(gamepad1.x);
        startTreatment.update(gamepad1.y);
        lockTarget.update(gamepad2.a);
        unlockTarget.update(gamepad2.x);
        toggle_arm_full.update(gamepad1.b);
        dpadUp.update(gamepad1.dpad_up);
        dpadDown.update(gamepad1.dpad_down);
        dpadLeft.update(gamepad1.dpad_left);
        dpadRight.update(gamepad1.dpad_right);
        leftbumper.update(gamepad1.left_bumper);
        rightbumper.update(gamepad1.right_bumper);
    }

    @Override
    public void stop() {
    }
    private void telemetry() {
        if(arm_full_toggle&& robot.isReadyToTreat()&&robot.isArm_ready()&&robot.isArmHomed()){
            telemetry.addLine("****** READY FULL CYCLE  ******");
        }
        else if(!arm_full_toggle && robot.isReadyToTreat()&&robot.isArm_ready()&&robot.isArmHomed())
        {
            telemetry.addLine("****** READY Move Arm ONLY ******");
        }
        telemetry.addLine("Telemetry: Geometry Verification");

        // Verify extension calibration: physical slide extension should match this inch value.
        telemetry.addLine("===== EXTENSION =====");
        telemetry.addData("Extension (in)", "%.2f", robot.getExtensionPosition());

        // Verify turret potentiometer conversion and that the shoulder pivot rides at the turret tip.
        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Angle", "%.2f", robot.turretAngle());
        telemetry.addData("Raw Pot", "%.3f", robot.turretPosition());
        telemetry.addData("Shoulder Pivot X", "%.2f", robot.shoulderPivotX());
        telemetry.addData("Shoulder Pivot Y", "%.2f", robot.shoulderPivotY());
        telemetry.addData("Radius", "%.2f", robot.shoulderPivotRadius());

        // Verify shoulder encoder geometry: angle should explain measured vertical height.
        telemetry.addLine("===== SHOULDER =====");
        telemetry.addData("Angle", "%.2f", robot.shoulderAngle());
        telemetry.addData("Height", "%.2f", robot.shoulderHeight());

        // Verify wrist model: wrist motion should change FK but not the pre-wrist arm position.
        telemetry.addLine("===== WRIST =====");
        telemetry.addData("Angle", "%.2f", robot.getWristAngle());
        telemetry.addData("Height", "%.2f", robot.getWristHeight());
        telemetry.addData("Length", "%.2f", robot.getWristLength());

        // Verify whole-arm geometry: current position is pre-wrist, FK position is nozzle/tip.
        telemetry.addLine("===== ARM =====");
        telemetry.addData("Current Position","(X) %.2f (Y) %.2f (Z) %.2f", robot.getCurrent_Arm_Position().x, robot.getCurrent_Arm_Position().y, robot.getCurrent_Arm_Position().z);
        telemetry.addData("FK Position","(X) %.2f (Y) %.2f (Z) %.2f", robot.getComputedEndEffectorPose().x, robot.getComputedEndEffectorPose().y, robot.getComputedEndEffectorPose().z);
        telemetry.addData("Delta","(X) %.2f (Y) %.2f (Z) %.2f",
                robot.getCurrent_Arm_Position().x - robot.getComputedEndEffectorPose().x,
                robot.getCurrent_Arm_Position().y - robot.getComputedEndEffectorPose().y,
                robot.getCurrent_Arm_Position().z - robot.getComputedEndEffectorPose().z);

        telemetry.addData("Auto", "Arm %s State %d Ready %s FullState %d", robot.isArm_automation() ? "YES" : "NO", robot.armAutoState(), robot.isArm_ready() ? "YES" : "NO", robot.getFullCycleState());
        telemetry.addData("Test AutoMove", "Active %s Phase %d", robot.isTestAutoMoveActive() ? "YES" : "NO", robot.getTestAutoMovePhase());
        // telemetry.addLine(String.format("[TAG] Detected %b x: %.2f y: %.2f z: %.2f", robot.isTagDetected(),robot.getTagLocation().x, robot.getTagLocation().y, robot.getTagLocation().z) );
        // telemetry.addData("[Vision] Locked", robot.isTreatmentTargetLocked() ? "YES" : "NO");
        // telemetry.addLine(String.format("[Arm](Homed) Shoulder %b Turret: %b Ext: %b", robot.shoulderIsHomed(),robot.turretIsHomed(),robot.isExtensionHome()) );
        // telemetry.addLine(String.format("[Arm](Busy) Sho: %b Tur: %b Ext: %b Wrst: %b Arm: %b ", robot.shoulderIsBusy(),robot.turretIsBusy(),robot.isBusyExtension(), robot.isWristBusy(),robot.isArmBusy()) );
        // telemetry.addLine(String.format("[Fog]  %b Count: %d Target: %d", robot.isCycling(),robot.getCyclecount(), robot.getCycleTarget()) );
        // telemetry.addLine(String.format("[Fog] Fan: %.0f Fog: %.0f Pump: %.0f", robot.getFanTime(),robot.getFogTime(), robot.getPumpTime()) );
        // telemetry.addData("[Ready] Treat: ",robot.isReadyToTreat()+" Arm: "+robot.isArmHomed()+" Imp: "+robot.isArmLocationLogicImprovement());
        // telemetry.addLine("  Controls Guide: Gamepad1");
        // telemetry.addLine("(A:Stop) (B:Full/Arm Toggle)");
        // telemetry.addLine("(X:Home Arm) (Y:Start Treatment)");
        // telemetry.addLine("(Dpad Up/Down: Cycle Count) (Dpad Left + Right- [Fog Time])");
        // telemetry.addLine("(Bumpers Buttons L+ R-: Fan Time)");
        // telemetry.addLine("  Controls Guide: Gamepad2");
        // telemetry.addLine("(A:Lock Target) (X:Unlock Target)");
        // telemetry.addLine("Left Stick Y: Extension  Right Stick Y: Shoulder");
        // telemetry.addLine("Dpad Up/Down: Wrist  Triggers: Turret");
        // telemetry.addLine("(B:Home Shoulder)");

        telemetry.addLine("Telemetry: (IK Compute Only)");
        addIKTelemetry("Target A", ikTestTargetA);
        addIKTelemetry("Target B", ikTestTargetB);
        addIKTelemetry("Target C", ikTestTargetC);
        addIKTelemetry("Target D", ikTestTargetD);


        telemetry.update();
    }

     private void addIKTelemetry(String label, Position target) {
         robot_system.IKSolution solution = robot.testIK(target);

         telemetry.addData("[IK] " + label + " Target", "(X) %.1f (Y) %.1f (Z) %.1f", target.x, target.y, target.z);
         telemetry.addData("[IK] " + label + " Accepted Y Window", "(%.1f, %.1f)", solution.acceptedYMin, solution.acceptedYMax);
         telemetry.addData("[IK] " + label + " Reachable", solution.reachable ? "YES" : "NO");
         telemetry.addData("[IK] " + label + " Selected Turret", solution.turretDegrees);
         telemetry.addData("[IK] " + label + " Turret Target", solution.turretDegrees);
         telemetry.addData("[IK] " + label + " Shoulder Angle", "%.1f", solution.shoulderAngle);
         telemetry.addData("[IK] " + label + " Extension Length", "%.1f", solution.extensionLength);
         telemetry.addData("[IK] " + label + " Wrist Angle", "%.1f", solution.wristAngle);
         telemetry.addData("[IK] " + label + " Selection Score", "%.3f", solution.selectionScore);
         telemetry.addData("[IK] " + label + " Candidate Count", solution.candidateCount);
         telemetry.addData("[IK] " + label + " Failure Reason", solution.failureReason);
     }
}
