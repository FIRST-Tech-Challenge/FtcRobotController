package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzArmUltimateGoal;
import org.firstinspires.ftc.teamcode.Controllers.Examples.HzGamepadClassicUltimateGoal;

/**
 * TeleOpMode for Team Hazmat<BR>
 *  Expected behavior.. trigger will pull arm back to park all the time.. To test only key pad, comment out trigger line.
 */
@TeleOp(name = "Test_Arm", group = "Test")
@Disabled
public class Test_Arm extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    HzGamepadClassicUltimateGoal hzGamepad;
    HzArmUltimateGoal hzArmUltimateGoal;

    public int keyCount = 0;

    @Override
    public void runOpMode() {
        hzArmUltimateGoal = new HzArmUltimateGoal(hardwareMap);
        hzGamepad = new HzGamepadClassicUltimateGoal(gamepad1,this);

        telemetry.addData("Hazmat TeleOp Mode", "v:1.0");

        //hzArm.initArm(this);
        hzArmUltimateGoal.initArm();
        hzArmUltimateGoal.initGrip();
        //Wait for pressing plan on controller
        waitForStart();
        keyCount=0;

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            //**** Arm Actions ****
            //Arm Rotation
            //COMMENT THIS LINE OUT TO TEST KEY PAD ONLY.
            //hzArm.moveArmByTrigger(hzGamepadClassic.getLeftTrigger(), this);


            if (hzGamepad.getButtonYPress()) {
                hzArmUltimateGoal.moveArmParkedPosition();
            }

            if (hzGamepad.getButtonBPress()) {
                hzArmUltimateGoal.moveArmHoldUpWobbleRingPosition();
            }

            if (hzGamepad.getButtonAPress()) {
                hzArmUltimateGoal.moveArmPickWobblePosition();
            }

            if (hzGamepad.getButtonXPress()) {
                hzArmUltimateGoal.moveArmPickRingPosition();
            }

            //gpArm.moveArmByTrigger(getLeftTrigger());
            if (hzGamepad.getLeftTriggerPress()) {
                hzArmUltimateGoal.moveArmByTrigger();
            }
            //Toggle Arm Grip actions
            if (hzGamepad.getLeftBumperPress()) {
                if(hzArmUltimateGoal.getGripServoState() == HzArmUltimateGoal.GRIP_SERVO_STATE.OPENED) {
                    hzArmUltimateGoal.closeGrip();
                } else if(hzArmUltimateGoal.getGripServoState() == HzArmUltimateGoal.GRIP_SERVO_STATE.CLOSED) {
                    hzArmUltimateGoal.openGrip();
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

        switch (hzArmUltimateGoal.getGripServoState()){
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
        telemetry.addData("armMotor.getCurrentPosition()", hzArmUltimateGoal.armMotor.getCurrentPosition());
        telemetry.addData("armMotor.getTargetPosition()", hzArmUltimateGoal.armMotor.getTargetPosition());

        telemetry.addData("armGripServo.getCurrentPosition()", hzArmUltimateGoal.armGripServo.getPosition());
        telemetry.addData("hzGamepad.getLeftTriggerPress()", hzGamepad.getLeftTriggerPress());

        switch (hzArmUltimateGoal.getCurrentArmPosition()){
            case PARKED: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "PARKED");
                break;
            }
            case DROP_WOBBLE_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "DROP_WOBBLE_RING");
                break;
            }
            case HOLD_UP_WOBBLE_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "HOLD_UP_WOBBLE_RING");
                break;
            }
            case PICK_WOBBLE:{
                telemetry.addData("hzArm.getCurrentArmPosition()", "PICK_WOBBLE");
                break;
            }
            case PICK_RING: {
                telemetry.addData("hzArm.getCurrentArmPosition()", "PICK_RING");
                break;
            }
        }
    }

}


