package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepadClassic;

/**
 * TeleOpMode for Team Hazmat<BR>
 *  Expected behavior.. trigger will pull arm back to park all the time.. To test only key pad, comment out trigger line.
 */
@TeleOp(name = "Test_Arm", group = "Test")
public class Test_Arm extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    HzGamepadClassic hzGamepad;
    Arm hzArm;

    public int keyCount = 0;

    @Override
    public void runOpMode() {
        hzArm = new Arm(hardwareMap);
        hzGamepad = new HzGamepadClassic(gamepad1,this);

        telemetry.addData("Hazmat TeleOp Mode", "v:1.0");

        //hzArm.initArm(this);
        hzArm.initArm();
        hzArm.initGrip();
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
                hzArm.moveArmParkedPosition();
            }

            if (hzGamepad.getButtonBPress()) {
                hzArm.moveArmHoldUpWobbleRingPosition();
            }

            if (hzGamepad.getButtonAPress()) {
                hzArm.moveArmPickWobblePosition();
            }

            if (hzGamepad.getButtonXPress()) {
                hzArm.moveArmPickRingPosition();
            }

            //gpArm.moveArmByTrigger(getLeftTrigger());
            if (hzGamepad.getLeftTriggerPress()) {
                hzArm.moveArmByTrigger2();
            }
            //Toggle Arm Grip actions
            if (hzGamepad.getLeftBumperPress()) {
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

        telemetry.addData("armGripServo.getCurrentPosition()", hzArm.armGripServo.getPosition());
        telemetry.addData("hzGamepad.getLeftTriggerPress()", hzGamepad.getLeftTriggerPress());

        switch (hzArm.getCurrentArmPosition()){
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


