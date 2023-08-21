package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.UpdatedClawbotHardware;
import org.firstinspires.ftc.teamcode.hardware.ImprovedGamepad;

/*
 * New Control Clawbot Teleop: Similar to NewClawBotTeleOp but it implements a new control method
 * For simplicity we will have 4 motor directions: RF(Right forward), RB(Right Backwards), LF(Left
 * forward), LB(Left Backwards)
 * We will also be referring to the controller axes in a similar way. There are 4 total axes:
 * left x, left y, right x, right y or simply LX, LY, RX, RY, with a "-"(negative sign) to indicate
 * which direction it is going in.
 * LY = RF && LF
 * LY- = RB && LB
 * LX = LF && RB
 * LX- = LB && RF
 * RY = RF && LF
 * RY- = RB && LB
 * RX = RB && LF
 * RX- = RF && LB
 *
 * The goal is to create a more natural feeling drive.
 * The arm of the robot uses a potentiometer and state machines
 * The clas of the robot is a servo and uses state machines
 */

//Created by Jerdenn25556 on 2/14/2023.

@TeleOp(name="OUTREACH New Control Clawbot Teleop", group="Outreach")
public class UpdatedClawbotTeleop extends OpMode {
    public static final double MINIMUM_LOW_ARM_VOLTAGE = 2.0;
    public static final double MAXIMUM_MEDIUM_ARM_VOLTAGE = 3.2;
    public static final double MAXIMUM_LOW_ARM_VOLTAGE = 2.7;
    public static final double TURN_WEIGHT = 1.25;
    public static final double STRAIGHT_POWER_WEIGHT = 2.0;
    private static UpdatedClawbotHardware.ArmState armState = UpdatedClawbotHardware.ArmState.GROUND;
    public UpdatedClawbotHardware.ClawState currentClawState = UpdatedClawbotHardware.ClawState.CLOSED;
    public ImprovedGamepad gamepad;
    public double voltage;
    public ImprovedGamepad masterGamepad;
    public ImprovedGamepad gamepadInControl;
    public ElapsedTime gamepadTimer = new ElapsedTime();
    public boolean override = false;
    public double rightPower = 0;
    public double leftPower = 0;


    public boolean gamepadOverride = false;

    UpdatedClawbotHardware robot = new UpdatedClawbotHardware();
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, gamepadTimer, "gamepad");
        masterGamepad = new ImprovedGamepad(gamepad2, gamepadTimer, "masterGamepad");
        gamepadInControl = null;
        robot.init(hardwareMap);
        telemetry();
    }

    @Override
    public void loop() {
        gamepad.update();
        masterGamepad.update();
        overrideControllerCheck();
        rightPower = (gamepadInControl.left_stick_y.getValue() / STRAIGHT_POWER_WEIGHT) + (gamepadInControl.right_stick_y.getValue() / STRAIGHT_POWER_WEIGHT) - (gamepadInControl.right_stick_x.getValue() / TURN_WEIGHT) - (gamepadInControl.left_stick_x.getValue() / TURN_WEIGHT);
        leftPower = (gamepadInControl.left_stick_y.getValue() / STRAIGHT_POWER_WEIGHT) + (gamepadInControl.right_stick_y.getValue() / STRAIGHT_POWER_WEIGHT) + (gamepadInControl.left_stick_x.getValue() / TURN_WEIGHT) + (gamepadInControl.right_stick_x.getValue() / TURN_WEIGHT);
        polarAxisValueTurning();
        robot.leftDrive.setPower(-leftPower);
        robot.rightDrive.setPower(-rightPower);
        stateMachineUpdate();
        voltageRegulation();
        armPositionUpdate();
        telemetry();
    }

    public void telemetry() {
        telemetry.addData("Master Control Start", "Start + B");
        telemetry.addData("Player Control Start", "Start + A");
        telemetry.addData("Forward/Backward", "Left and Right game stick");
        telemetry.addData("Open/Close claw toggle", "B");
        telemetry.addData("Open claw", "Left Bumper/Trigger");
        telemetry.addData("Close claw", "Right Bumper/Trigger");
        telemetry.addData("Move Arm Up", "D-pad up");
        telemetry.addData("Move Arm Down", "D-pad down");
        telemetry.addData("Master Gamepad Override", "X");
        telemetry.addData("Override", override);
        telemetry.update();
    }


    public void polarAxisValueTurning() {
        /*
         * With the way the turning use to work without this method, if you held one stick all the way
         * forward on the y-axis and held the other stick all the way back on the y-axis, the robot
         * would stay stationary. If you are confused by this in the future, remove the call of this
         * method and loop, see how it controls in the situation above, then add it back.
         */

        if (gamepadInControl.left_stick_y.getValue() > .9 && gamepadInControl.right_stick_y.getValue() < -.9) {
            if ((gamepadInControl.left_stick_x.getValue() < .25 && gamepadInControl.left_stick_x.getValue() > -.25) && (gamepadInControl.right_stick_x.getValue() < .25 && gamepadInControl.right_stick_x.getValue() > -.25) ) {
                leftPower = 1;
                rightPower = -1;
            }
        }

        if (gamepadInControl.right_stick_y.getValue() > .9 && gamepadInControl.left_stick_y.getValue() < -.9 ) {
            if ((gamepadInControl.left_stick_x.getValue() < .25 && gamepadInControl.left_stick_x.getValue() > -.25) && (gamepadInControl.right_stick_x.getValue() < .25 && gamepadInControl.right_stick_x.getValue() > -.25)) {
                leftPower = -1;
                rightPower = 1;
            }
        }
    }

    public void voltageRegulation() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        double scaleFactor = 12/result;
        voltage = scaleFactor * robot.potentiometer.getVoltage();
    }

    public void armPositionUpdate() {
        if ((gamepadInControl.right_stick_y.getValue() != 0 || gamepadInControl.left_stick_y.getValue() != 0 || gamepadInControl.left_stick_x.getValue() != 0 || gamepadInControl.right_stick_x.getValue() != 0) && (armState == UpdatedClawbotHardware.ArmState.GROUND || armState == UpdatedClawbotHardware.ArmState.LOW) ) {
            armState = UpdatedClawbotHardware.ArmState.LOW;
        }

        if (armState == UpdatedClawbotHardware.ArmState.MEDIUM && voltage > MINIMUM_LOW_ARM_VOLTAGE) {
            robot.arm.setPower(0.5);
        } else if (armState == UpdatedClawbotHardware.ArmState.LOW && voltage < MAXIMUM_MEDIUM_ARM_VOLTAGE) {
            robot.arm.setPower(-0.2);
        } else {
            robot.arm.setPower(0);
        }

        if ((rightPower != 0 || leftPower != 0) && voltage > MAXIMUM_LOW_ARM_VOLTAGE) {
            robot.arm.setPower(0.3);
        }
    }

    public void stateMachineUpdate() {
        switch(currentClawState){
            case OPEN:
                robot.claw.setPosition(UpdatedClawbotHardware.CLAW_OPEN_POSITION);
                if(gamepadInControl.b.isInitialPress()){
                    currentClawState = UpdatedClawbotHardware.ClawState.CLOSED;
                } else if (gamepadInControl.right_bumper.getValue() || gamepadInControl.right_trigger.getValue() > 0) {
                    currentClawState = UpdatedClawbotHardware.ClawState.CLOSED;
                }
                break;
            case CLOSED:
                robot.claw.setPosition(UpdatedClawbotHardware.CLAW_CLOSED_POSITION);
                if(gamepadInControl.b.isInitialPress()){
                    currentClawState = UpdatedClawbotHardware.ClawState.OPEN;
                } else if (gamepadInControl.left_bumper.getValue() || gamepadInControl.left_trigger.getValue() > 0) {
                    currentClawState = UpdatedClawbotHardware.ClawState.CLOSED;
                }
                break;
        }

        switch (armState) {
            case GROUND:
            case LOW:
                if (gamepadInControl.dpad_up.isInitialPress()) {
                    armState = UpdatedClawbotHardware.ArmState.MEDIUM;
                }
                break;
            case MEDIUM:
                if (gamepadInControl.dpad_down.isInitialPress()) {
                    armState = UpdatedClawbotHardware.ArmState.LOW;
                }
                break;
        }
    }

    public void overrideControllerCheck() {
        if (masterGamepad.areButtonsActive()) {
            override = true;
        } else {
            override = false;
        }

        if(masterGamepad.x.isInitialPress() && gamepadOverride){
            gamepadOverride = false;
        }else if(masterGamepad.x.isInitialPress() && !gamepadOverride){
            gamepadOverride = true;
        }

        if(gamepadOverride || masterGamepad.areButtonsActive()){
            gamepadInControl = masterGamepad;
        }else{
            gamepadInControl = gamepad;
        }
    }
}
