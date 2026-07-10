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
    ButtonBlock lockTarget, lockManualTarget, unlockTarget;
    ButtonBlock dpadUp, dpadDown, dpadLeft, dpadRight;
    ButtonBlock leftbumper, rightbumper;
    boolean arm_full_toggle = true;
    int yPosTarget = 0;

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
                    // After lock-target automation finishes, X returns that
                    // test move home. Otherwise X runs the normal arm-home path.
                    if (robot.isTestAutoMoveCompleted()) {robot.testReturnHome();}
                    else {robot.armHome();}
                });
        startTreatment = new ButtonBlock()
                .onTrue(()->{robot.startTreatment(arm_full_toggle);});
        lockTarget = new ButtonBlock()
                .onTrue(() -> {robot.lockCurrentFilteredTarget(true);});
        lockManualTarget = new ButtonBlock()
                .onTrue(() -> {robot.lockManualTestTarget(true);});
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

        double shoulderStick = gamepad2.right_stick_y;
        if (Math.abs(shoulderStick) < 0.05) {
        shoulderStick = 0;
        }

        if (!robot.shoulderIsBusy()) {
                robot.shoulderSetPower(-shoulderStick);
        } else if (Math.abs(shoulderStick) > 0) {
                robot.shoulderStop();
                robot.shoulderSetPower(-shoulderStick);
        }
        // if (!robot.shoulderIsBusy()) robot.shoulderSetPower(-gamepad2.right_stick_y);
        // else if (Math.abs(gamepad2.right_stick_y) > 0) {
        //     robot.shoulderStop();
        //     robot.shoulderSetPower(-gamepad2.right_stick_y);
        // }
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
        lockManualTarget.update(gamepad2.y);
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

        // Verify continuous turret targeting: formula pot should match the raw pot when the turret finishes.
        telemetry.addLine("===== TURRET TARGET DEBUG =====");
        telemetry.addData("Busy", robot.turretIsBusy() ? "YES" : "NO");
        telemetry.addData("IK Target Degrees", "%.1f", robot.getTestTurretTargetDegrees());
        telemetry.addData("Formula Target Pot", "%.3f", robot.getTestTurretTargetPot());
        telemetry.addData("Current Raw Pot", "%.3f", robot.turretPosition());
        telemetry.addData("Formula Pot Error", "%.3f", Math.abs(robot.turretPosition() - robot.getTestTurretTargetPot()));
        telemetry.addData("Turret Internal Target Pot", "%.3f", robot.turretTargetPosition());

        // Verify shoulder encoder geometry: angle should explain measured vertical height.
        telemetry.addLine("===== SHOULDER =====");
        telemetry.addData("Angle", "%.2f", robot.shoulderAngle());
        telemetry.addData("Height", "%.2f", robot.shoulderHeight());
        telemetry.addData("Target Angle", "%.2f", robot.shoulderTargetAngle());
        telemetry.addData("Error Deg", "%.2f", robot.shoulderTargetErrorDegrees());
        telemetry.addData("Raw Current", "%.0f", robot.shoulderRaw());
        telemetry.addData("Raw Target", "%.0f", robot.shoulderRawTarget());
        telemetry.addData("Error Ticks", "%.0f", robot.shoulderTargetErrorTicks());
        telemetry.addData("Power", "%.2f", robot.shoulderPower());

        // Verify wrist model: raw values come from linkage angle; level values are used by IK/FK.
        telemetry.addLine("===== WRIST =====");
        telemetry.addData("Angle", "%.2f", robot.getWristAngle());
        telemetry.addData("Raw Height", "%.2f", robot.getWristHeight());
        telemetry.addData("Raw Length", "%.2f", robot.getWristLength());
        telemetry.addData("Level FK Height", "%.2f", robot.getLevelWristHeight());
        telemetry.addData("Level FK Length", "%.2f", robot.getLevelWristLength());
        telemetry.addData("Level Calc Angle", "%.2f", robot.getWristCalc());
        telemetry.addData("Servo Target Angle", "%.2f", robot.getWristTargetAngle());
        telemetry.addData("Servo Raw Target", "%.3f", robot.getWristRawTargetPosition());

        // Verify whole-arm geometry: current position is pre-wrist, FK position is nozzle/tip.
        telemetry.addLine("===== ARM =====");
        telemetry.addData("Current Position","(X) %.2f (Y) %.2f (Z) %.2f", robot.getCurrent_Arm_Position().x, robot.getCurrent_Arm_Position().y, robot.getCurrent_Arm_Position().z);
        telemetry.addData("FK Position","(X) %.2f (Y) %.2f (Z) %.2f", robot.getComputedEndEffectorPose().x, robot.getComputedEndEffectorPose().y, robot.getComputedEndEffectorPose().z);
        telemetry.addData("Delta","(X) %.2f (Y) %.2f (Z) %.2f",
                robot.getCurrent_Arm_Position().x - robot.getComputedEndEffectorPose().x,
                robot.getCurrent_Arm_Position().y - robot.getComputedEndEffectorPose().y,
                robot.getCurrent_Arm_Position().z - robot.getComputedEndEffectorPose().z);

        robot_system.IKSolution lastIK = robot.getLastIKSolution();

        // Verify locked-target IK: if lock does not move, this explains which geometry limit blocked it.
        telemetry.addLine("===== IK DEBUG =====");
        telemetry.addData("Reachable", lastIK.reachable ? "YES" : "NO");
        telemetry.addData("Move Allowed", lastIK.reachable ? "YES" : "NO");
        telemetry.addData("Failure", lastIK.failureReason == null ? "No IK run yet" : lastIK.failureReason);
        telemetry.addData("Candidate Count", lastIK.candidateCount);
        telemetry.addData("Selection Score", "%.3f", lastIK.selectionScore);

        // Verify the camera/target point used by the solver.
        telemetry.addLine("===== IK TARGET =====");
        telemetry.addData("Target Source", robot.getLockedTargetSource());
        telemetry.addData("Manual Test Target", "(X) %.2f (Y) %.2f (Z) %.2f",
                robot.getManualTestTarget().x,
                robot.getManualTestTarget().y,
                robot.getManualTestTarget().z);
        telemetry.addData("Target", "(X) %.2f (Y) %.2f (Z) %.2f", lastIK.targetX, lastIK.targetY, lastIK.targetZ);
        telemetry.addData("Camera X Correction", "%.2f", robot.getIKCameraXCorrection());
        telemetry.addData("Camera Z Correction", "%.2f", robot.getIKCameraZCorrection());
        telemetry.addData("Accepted Y Window", "%.2f to %.2f", lastIK.acceptedYMin, lastIK.acceptedYMax);

        // Best answer from the solver, even when it fails a safety/reachability check.
        telemetry.addLine("===== IK BEST ANSWER =====");
        telemetry.addData("Selected Turret", "%.1f", lastIK.turretDegrees);
        telemetry.addData("Selected Turret Pot", "%.3f", lastIK.turretPotTarget);
        telemetry.addData("Selected Shoulder Angle", "%.2f", lastIK.shoulderAngle);
        telemetry.addData("Level Wrist Command", "%.2f", lastIK.wristAngle);
        telemetry.addData("Selected Extension", "%.2f", lastIK.extensionLength);
        telemetry.addData("Needed Raw Extension", "%.2f", lastIK.rawExtensionLength);
        telemetry.addData("Target Error", "(Y) %.2f (Z) %.2f", lastIK.targetErrorY, lastIK.targetErrorZ);

        // Verify the chosen robot commands before they are sent to the subsystems.
        telemetry.addLine("===== IK COMMAND TARGETS =====");
        telemetry.addData("Turret Target Degrees", "%.1f", robot.getTestTurretTargetDegrees());
        telemetry.addData("Turret Target Pot", "%.3f", robot.getTestTurretTargetPot());
        telemetry.addData("Extension Target", "%.2f", robot.getTestExtensionTarget());
        telemetry.addData("Shoulder Target", "%.2f", robot.getTestShoulderTargetAngle());
        telemetry.addData("Wrist Level Command", "%.2f", robot.getTestWristTargetAngle());

        // Verify the geometry terms used inside IK before clamping.
        telemetry.addLine("===== IK GEOMETRY BREAKDOWN =====");
        telemetry.addData("Turret Offset", "(X) %.2f (Y) %.2f", lastIK.turretOffsetX, lastIK.turretOffsetY);
        telemetry.addData("Shoulder", "(Length) %.2f (Height) %.2f", lastIK.shoulderLength, lastIK.shoulderHeight);
        telemetry.addData("Wrist", "(Length) %.2f (Height) %.2f", lastIK.wristLength, lastIK.wristHeight);
        telemetry.addData("Wrist X Included", "%.2f", lastIK.wristLength);
        telemetry.addData("IK Residual", "(Y) %.2f (Z) %.2f", lastIK.targetErrorY, lastIK.targetErrorZ);
        telemetry.addData("Raw Extension", "%.2f", lastIK.rawExtensionLength);

        // Use this section to find which subsystem is blocking the next auto phase.
        telemetry.addLine("===== AUTOMATION STATE DEBUG =====");
        telemetry.addData("Test AutoMove", "Active %s Done %s Phase %d",
                robot.isTestAutoMoveActive() ? "YES" : "NO",
                robot.isTestAutoMoveCompleted() ? "YES" : "NO",
                robot.getTestAutoMovePhase());
        telemetry.addData("Subsystem Busy", "Tur %s Sho %s Wri %s Ext %s Arm %s",
                robot.turretIsBusy() ? "YES" : "NO",
                robot.shoulderIsBusy() ? "YES" : "NO",
                robot.isWristBusy() ? "YES" : "NO",
                robot.isBusyExtension() ? "YES" : "NO",
                robot.isArmBusy() ? "YES" : "NO");
        telemetry.addData("Other Automation", "Arm %s Full %s Cycle %s",
                robot.isArm_automation() ? "YES" : "NO",
                robot.isFullCycleAutomation() ? "YES" : "NO",
                robot.isCycling() ? "YES" : "NO");

        telemetry.addData("Auto", "Arm %s State %d Ready %s FullState %d", robot.isArm_automation() ? "YES" : "NO", robot.armAutoState(), robot.isArm_ready() ? "YES" : "NO", robot.getFullCycleState());
        telemetry.addData("Test AutoMove", "Active %s Done %s Phase %d",
                robot.isTestAutoMoveActive() ? "YES" : "NO",
                robot.isTestAutoMoveCompleted() ? "YES" : "NO",
                robot.getTestAutoMovePhase());
        telemetry.addLine(String.format("[TAG] Detected %b x: %.2f y: %.2f z: %.2f", robot.isTagDetected(),robot.getTagLocation().x, robot.getTagLocation().y, robot.getTagLocation().z) );
        telemetry.addData("[Vision] Locked", robot.isTreatmentTargetLocked() ? "YES" : "NO");
        telemetry.addLine(String.format("[Arm](Homed) Shoulder %b Turret: %b Ext: %b", robot.shoulderIsHomed(),robot.turretIsHomed(),robot.isExtensionHome()) );


        telemetry.update();
    }
}
