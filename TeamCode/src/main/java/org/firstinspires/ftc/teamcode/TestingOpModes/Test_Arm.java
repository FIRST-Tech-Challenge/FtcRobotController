package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepadClassic;

/**
 * TeleOpMode for Team Hazmat<BR>
 */
@TeleOp(name = "Test_Arm", group = "Test")
public class Test_Arm extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    HzGamepadClassic hzGamepadClassic;
    Arm hzArm;

    public int keyCount = 0;

    @Override
    public void runOpMode() {
        hzArm = new Arm(hardwareMap);
        hzGamepadClassic = new HzGamepadClassic(gamepad1,this);

        telemetry.addData("Hazmat TeleOp Mode", "v:1.0");

        hzArm.initArm(this);
        hzArm.initGrip();
        //Wait for pressing plan on controller
        waitForStart();
        keyCount=0;

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            //**** Arm Actions ****
            //Arm Rotation
              hzArm.moveArmByTrigger(hzGamepadClassic.getLeftTrigger(), this);

            if (hzGamepadClassic.getButtonYPress()) {
                hzArm.moveArmParkedPosition();
                }

            if (hzGamepadClassic.getButtonBPress()) {
                hzArm.moveArmDropWobbleGoalPosition();
            }

            if (hzGamepadClassic.getButtonAPress()) {
                hzArm.moveArmHoldWobbleRingPosition();
            }

            if (hzGamepadClassic.getButtonXPress()) {
                hzArm.moveArmRingPosition();
            }

            //Toggle Arm Grip actions
            if (hzGamepadClassic.getLeftBumperPress()) {
                if(hzArm.getGripServoState() == Arm.GRIP_SERVO_STATE.OPENED) {
                    hzArm.closeGrip();
                } else if(hzArm.getGripServoState() == Arm.GRIP_SERVO_STATE.CLOSED) {
                    hzArm.openGrip();
                }
            }

            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
        }
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);

        switch (hzArm.getGripServoState()){
            case OPENED  : {
                telemetry.addData("hzArm.getGripServoState()", "OPENED");
                break;
            }
            case CLOSED: {
                telemetry.addData("hzArm.getGripServoState()", "CLOSED");
                break;
            }
        }

        telemetry.addData("7:05","11/23");
        telemetry.addData("armMotor.getCurrentPosition()", hzArm.armMotor.getCurrentPosition());
        telemetry.addData("armMotor.getTargetPosition()", hzArm.armMotor.getTargetPosition());

        telemetry.addData("triggerPositionCount", hzArm.triggerPositionCount);
        telemetry.addData("armGripServo.getCurrentPosition()", hzArm.armGripServo.getPosition());

        switch (hzArm.getCurrentArmPosition()){
            case ARM_PARKED_POSITION : {
                telemetry.addData("hzArm.getCurrentArmPosition()", "ARM_MOTOR_PARKED_POSITION");
                break;
            }
            case ARM_DROP_WOBBLE_GOAL_POSITION : {
                telemetry.addData("hzArm.getCurrentArmPosition()", "ARM_DROP_WOBBLE_GOAL_POSITION");
                break;
            }
            case ARM_HOLD_WOBBLE_GOAL_POSITION : {
                telemetry.addData("hzArm.getCurrentArmPosition()", "ARM_HOLD_WOBBLE_GOAL_POSITION");
                break;
            }
            case ARM_RING_POSITION : {
                telemetry.addData("hzArm.getCurrentArmPosition()", "ARM_RING_POSITION");
                break;
            }
        }
    }

}


