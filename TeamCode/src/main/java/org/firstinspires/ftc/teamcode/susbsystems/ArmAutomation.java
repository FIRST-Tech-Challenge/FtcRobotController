package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.teamcode.Constants.EXTENSION_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.SHOULDER_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.SHOULDER_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.TAG_ID_1_Y_MAX;
import static org.firstinspires.ftc.teamcode.Constants.TAG_ID_1_Y_MIN;
import static org.firstinspires.ftc.teamcode.Constants.TAG_ID_1_Z_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.TURRENT_TO_CAMERA;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Arm automation state machines.
 *
 * robot_system remains the public facade. This class owns the multi-step logic
 * so later true IK and home-return tuning can be done away from hardware setup.
 */
class ArmAutomation {
    private final robot_system robot;

    ArmAutomation(robot_system robot) {
        this.robot = robot;
    }

    void updateArmPositionAndHome() {
        Position turretOffset = RobotGeometry.turretOffset(robot.turret.getNewcurrentAngleDegrees());
        robot.arm_position = ForwardKinematics.computeArmPosition(
                robot.extension.GetPos(),
                robot.shoulder.getExtension(),
                robot.shoulder.GetHeight(),
                turretOffset);

        if (robot.arm_automation) {
            if (!robot.shoulder.IsBusy() && !robot.turret.IsBusy() && !robot.extension.IsBusy() && robot.arm_state == 0) {
                robot.arm_automation = false;
            }
        }

        if(robot.shoulder.GetRawPos() <25 && robot.shoulder.GetRawPos() > -25 && robot.turret.GetRawPos() >.310 && robot.extension.isAtHome()){
            if(robot.arm_ready){robot.arm_homed = true;}
        }
        else robot.arm_homed = false;
    }

    void armReady() {
        if (robot.shoulder.Homed() && robot.turret.Homed() && robot.extension.isAtHome()) {
            robot.arm_ready = true;
        }
    }

    void stopArmToPosition() {
        robot.arm_automation = false;
        robot.arm_homing = false;
        robot.arm_state = 0;
        robot.treatmentTargetLocked = false;
        robot.testAutoMoveActive = false;
        robot.testAutoMovePhase = 0;
        robot.wrist.Stop();
        robot.turret.Stop();
        robot.shoulder.Stop();
        robot.turret.Stop();
        robot.extension.Stop();
    }

    void initTestLockedTargetAutoMove() {
        // Temporary lock-test move: this is intentionally simple until true IK replaces it.
        robot.testAutoMoveActive = true;
        robot.testAutoMovePhase = 1;
        robot.testTurretTargetDegrees = 135;
        robot.testExtensionTarget = RobotGeometry.clamp(robot.extension.GetPos() + 5, 0, EXTENSION_MAX_POSITION);
        robot.testShoulderTargetAngle = clampShoulderAngle(robot.shoulder.GetPos() + 15);
        robot.testWristTargetAngle = RobotGeometry.clamp(robot.wrist.GetAngle() - 30, 36, 180);
    }

    void testStopAutoMove() {
        robot.testAutoMoveActive = false;
        robot.testAutoMovePhase = 0;
        robot.wrist.Stop();
        robot.turret.Stop();
        robot.shoulder.Stop();
        robot.extension.Stop();
    }

    void testLockedTargetAutoMove() {
        if (robot.arm_automation || robot.fullCycleAutomation || robot.cycling) {
            testStopAutoMove();
            return;
        }

        if (robot.testAutoMovePhase == 1 && !robot.turret.IsBusy()) {
            robot.moveTurret(robot.testTurretTargetDegrees);
            robot.testAutoMovePhase = 2;
        }
        if (robot.testAutoMovePhase == 2 && !robot.turret.IsBusy()) {
            robot.extension.GoTo(robot.testExtensionTarget);
            robot.testAutoMovePhase = 3;
        }
        if (robot.testAutoMovePhase == 3 && !robot.extension.IsBusy()) {
            robot.shoulder.GoToAngle(robot.testShoulderTargetAngle);
            robot.testAutoMovePhase = 4;
        }
        if (robot.testAutoMovePhase == 4 && !robot.shoulder.IsBusy()) {
            robot.wrist.GoToAngle(robot.testWristTargetAngle);
            robot.testAutoMovePhase = 5;
        }
        if (robot.testAutoMovePhase == 5 && !robot.wrist.IsBusy()) {
            robot.testAutoMoveActive = false;
            robot.testAutoMovePhase = 0;
        }
    }

    void initArmToHome() {
        if(robot.arm_ready && !robot.arm_is_busy) {
            robot.arm_automation = true;
            robot.arm_homing = true;
            robot.extension.StartHome();
            robot.wrist.SetPos(.9);
            robot.arm_state = 1;
        }
    }

    boolean initArmToPosition(Position target, int degrees) {
        boolean validPosition = false;
        if (robot.arm_ready&&!robot.arm_is_busy&& robot.arm_homed) {
            robot.arm_state = 1;
            robot.arm_automation = true;
            if(target.x > -20) stopArmToPosition();
            if(target.x < -38) stopArmToPosition();
            if(target.y > 15) stopArmToPosition();
            if(target.y <-3) stopArmToPosition();
            if(target.z <SHOULDER_MIN_HEIGHT) stopArmToPosition();
            if(target.z > SHOULDER_MAX_HEIGHT) stopArmToPosition();
            if(robot.arm_automation){
                if (degrees == 180) {
                    robot.turret.GoTo(Constants.ONEEIGHTY_DEGREES);
                    validPosition = true;
                } else if (degrees == 90) {
                    robot.turret.GoTo(Constants.NINETY_DEGREES);
                    validPosition = true;
                } else if (degrees == 135) {
                    robot.turret.GoTo(Constants.ONETHIRTYFIVE_DEGREES);
                    validPosition = true;
                } else if (degrees == 225) {
                    robot.turret.GoTo(Constants.TWOTWENTYFIVE_DEGREES);
                    validPosition = true;
                } else if (degrees == 45) {
                    robot.turret.GoTo(Constants.FORTYFIVE_DEGREES);
                    validPosition = true;
                } else {stopArmToPosition();}
            }
        }
        return validPosition;
    }

    void armToPosition() {
        if(!robot.arm_is_busy&&!robot.arm_homing) {
            if (robot.arm_state == 1 &&!robot.turret.IsBusy()) {
                robot.arm_state = 2;
                robot.extension.GoTo(2);
                robot.wrist.SetTargetPos(.9);
            }
            if (robot.arm_state == 2 && !robot.extension.IsBusy()) {
                robot.arm_state = 3;
                robot.shoulder.GoToHeight(robot.target_position.z - TURRENT_TO_CAMERA.z);
            }
            if (robot.arm_state == 3 && !robot.shoulder.IsBusy()) {
                robot.arm_state = 4;
                robot.wrist.SetTargetPos(.55);
            }
            if (robot.arm_state == 4 && !robot.wrist.IsBusy()) {
                robot.arm_state = 5;
                robot.extension_target = Math.abs(robot.target_position.x)-(Math.abs(robot.wrist.GetLength())+Math.abs(robot.arm_position.x)+1);
                robot.extension.GoTo(robot.extension_target);
            }
            if (robot.arm_state == 5 && !robot.extension.IsBusy()) {
                robot.arm_state = 0;
                robot.arm_automation = false;
                robot.treatmentTargetLocked = false;
            }
        }
        else if(!robot.arm_is_busy && robot.arm_homing){
            if (robot.arm_state == 1 &&!robot.extension.IsBusy()) {
                robot.arm_state = 2;
                robot.shoulder.GoToEncoderPosition(0);
                robot.shoulder.Update();
            }
            if (robot.arm_state == 2 && !robot.shoulder.IsBusy()) {
                robot.arm_state = 3;
                robot.turret.GoTo(Constants.TURRET_MAX_POSITION);
            }
            if (robot.arm_state == 3 && !robot.turret.IsBusy()) {
                robot.arm_state = 0;
                robot.arm_automation = false;
                robot.arm_homing = false;
            }
        }
    }

    void startFullCycle(Position target) {
        if(robot.arm_ready && robot.arm_homed){
            robot.fullCycleAutomation = true;
            robot.fullCycleState = 1;
            robot.arm_last_position_bad = initArmToPosition(target,robot.target_degrees);
        }
    }

    void stopFullCycle() {
        robot.fullCycleAutomation = false;
        robot.fullCycleState = 0;
        stopArmToPosition();
        robot.stopFogCycle();
    }

    void fullCycle() {
        if(robot.fullCycleState == 1 && !robot.arm_automation){
            robot.fullCycleState++;
            robot.init_cycle();
        }
        if(robot.fullCycleState == 2 && !robot.cycling){
            robot.fullCycleState++;
            robot.pump.TurnOff();
            robot.fan.TurnOff();
            robot.fogger.TurnOff();
            initArmToHome();
        }
        if(robot.fullCycleState ==3 && !robot.arm_automation){
            robot.fullCycleState = 0;
            robot.fullCycleAutomation = false;
        }
    }

    void startTreatment(boolean armOrTreat) {
        if(robot.isReadyToTreat&&robot.arm_homed){
            if(armOrTreat)
                startFullCycle(robot.target_position);
            else
                robot.arm_last_position_bad = initArmToPosition(robot.target_position, robot.target_degrees);
        }
    }

    boolean tagToArmTest() {
        boolean ready = false;
        // Tag ID 1 is left. These checks produce a reachable target for the current simple automation.
        if(robot.tagID ==1 && robot.arm_homed) {
            robot.target_position.z = robot.tagLocation.z + TAG_ID_1_Z_OFFSET - robot.wrist.GetHeight();
            if(robot.target_position.z > SHOULDER_MAX_HEIGHT || robot.target_position.z < SHOULDER_MIN_HEIGHT) {
                robot.target_position.z = 0;
                return ready;
            }
            if(robot.tagLocation.y > RobotGeometry.turretTargetY(90)+TAG_ID_1_Y_MAX) {
                robot.target_position.y = 0;
                return ready;
            }
            if(robot.tagLocation.y < RobotGeometry.turretTargetY(225)+TAG_ID_1_Y_MIN) {
                robot.target_position.y =0;
                return ready;
            }
            robot.target_position.y = robot.tagLocation.y;
            if(robot.tagLocation.x > -30 ) {
                robot.target_position.x = 0;
                return ready;
            }
            if(robot.tagLocation.x < -44) {
                robot.target_position.x = 0;
                return ready;
            }
            robot.target_position.x = robot.tagLocation.x;
            robot.armLocationLogicImprovement = false;

            if(robot.tagLocation.y < RobotGeometry.turretTargetY(90)+TAG_ID_1_Y_MAX && robot.tagLocation.y > RobotGeometry.turretTargetY(135)+TAG_ID_1_Y_MAX){
                ready = true;
                robot.target_degrees = 90;
                robot.target_position.y = RobotGeometry.turretTargetY(90);
                robot.target_position.x = robot.tagLocation.x+2;
            }
            if(robot.tagLocation.y < TAG_ID_1_Y_MAX+TURRENT_TO_CAMERA.y && robot.tagLocation.y > TAG_ID_1_Y_MIN+TURRENT_TO_CAMERA.y && robot.tagLocation.x < RobotGeometry.turretTargetX(180)){
                ready = true;
                robot.target_degrees = 180;
                robot.target_position.y = RobotGeometry.turretTargetY(180);
                robot.target_position.x = robot.tagLocation.x+2;
            }
            if(robot.tagLocation.y < RobotGeometry.turretTargetY(135)+TAG_ID_1_Y_MAX && robot.tagLocation.y > RobotGeometry.turretTargetY(180)+TAG_ID_1_Y_MAX){
                ready = true;
                robot.target_degrees = 135;
                robot.target_position.y = RobotGeometry.turretTargetY(135);
                robot.target_position.x = robot.tagLocation.x+2;
            }
            if(robot.tagLocation.y < RobotGeometry.turretTargetY(180)+TAG_ID_1_Y_MAX && robot.tagLocation.y > RobotGeometry.turretTargetY(225)+TAG_ID_1_Y_MAX){
                ready = true;
                robot.target_degrees = 225;
                robot.target_position.y = RobotGeometry.turretTargetY(225);
                robot.target_position.x = robot.tagLocation.x+2;
            }
            if(!ready) robot.armLocationLogicImprovement = true;
        }
        return ready;
    }

    private double clampShoulderAngle(double angle) {
        return RobotGeometry.clamp(
                angle,
                RobotGeometry.shoulderMinAngle(SHOULDER_MIN_HEIGHT),
                RobotGeometry.shoulderMaxAngle(SHOULDER_MAX_HEIGHT));
    }
}
