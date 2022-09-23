package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PotentiometerSystem;
import org.firstinspires.ftc.teamcode.components.TurnTableSystem;
import org.firstinspires.ftc.teamcode.helpers.TeamState;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public abstract class DriveOpMode extends BaseOpMode {

    TeamState teamState;
    int aBoolean = 0; // arm keep movin up-top/bottom
    int bBoolean = 0; //we ain't moving
    int cBoolean = 0;
    int dBoolean = 0;

    public void init(TeamState teamState) {
        super.init();
        this.teamState = teamState;
    }

    @Override
    public void loop() {
        telemetry.addData("Turntable set values", "dpad right/left contr 2");
        telemetry.addData("turntable turn", "bumpers contr 1");
        telemetry.addData("move arm up/down", "dpad up/down contr 2");
        telemetry.addData("intake", "right bumper cntrl 2");
        telemetry.addData("outtake", "left bumper contrl 2");
        telemetry.addData("carousel", "right stick button cntrl 2");
        telemetry.addData("bruh", armSystem.getSensorAsAnalogInput0());
        telemetry.addData("position", imuSystem.imu.getPosition());

       /* telemetry.addData("See: ", armSystem.getSensorAsAnalogInput0());
        telemetry.addData("rotatorMotor", turnTableSystem.getPosition());
        telemetry.addData("elevator-motor", armSystem.getElevatorMotor().getCurrentPosition());
        telemetry.addData("motor-back-left", driveSystem.motors.get(DriveSystem.MotorNames.BACKLEFT).getCurrentPosition());
        telemetry.addData("motor-front-left", driveSystem.motors.get(DriveSystem.MotorNames.FRONTLEFT).getCurrentPosition());
        telemetry.addData("motor-back-right", driveSystem.motors.get(DriveSystem.MotorNames.BACKRIGHT).getCurrentPosition());
        telemetry.addData("motor-front-right", driveSystem.motors.get(DriveSystem.MotorNames.FRONTRIGHT).getCurrentPosition());*/

        float rx = (float) Math.pow(gamepad1.right_stick_x, 3);
        float lx = (float) Math.pow(gamepad1.left_stick_x, 3);
        float ly = (float) Math.pow(gamepad1.left_stick_y, 3);

        if (gamepad2.y){ // top-most
            //armSystem.moveToPosition(ArmSystem.LEVEL_TOP);
            telemetry.addData("set to move up", "top-button clicked:");
            aBoolean = 1; // set it to go up
            bBoolean = 0;
        }
        else if (gamepad2.a){ // bottom-most
            //armSystem.moveToPosition(ArmSystem.LEVEL_BOTTOM);
            telemetry.addData("set to move down", "bottom-button clicked:");
            aBoolean = -1; //set it to go down
            bBoolean = 0;
        }
        else{
            // if not being pressed don't do anything to the booleans
        }

        if (gamepad2.x){ //if x is pressed (carousel)
            telemetry.addData("set to move to carousel", "carousel-button clicked:");
            aBoolean = 0;
            bBoolean = 1; // 1 represents carousel
            cBoolean = armSystem.getSensorAsAnalogInput0() < ArmSystem.LEVEL_CAROUSEL ? 1 : -1; // up or down of the carousel
        }
        else if (gamepad2.b){ // (bottom level)
            telemetry.addData("set to move to carousel", "carousel-button clicked:");
            aBoolean = 0;
            bBoolean = -1; // -1 represents bottom level
            dBoolean = armSystem.getSensorAsAnalogInput0() < ArmSystem.LEVEL_BOTTOM ? 1 : -1; // up or down of the shared-level
        }
        else{

        }

        /*** DPAD/MANUAL CONTROLS **/

        if (gamepad2.dpad_up /*&& armSystem.notTooHigh()*/) {
            telemetry.addLine("can go up");
            aBoolean = 0; // arm keep movin up-top/bottom
            bBoolean = 0; //we ain't moving
            cBoolean = 0;
            dBoolean = 0;
            armSystem.moveUp();
            telemetry.addData("ACTIVE", "armSystem up");
        } else if (gamepad2.dpad_down /*&& armSystem.notTooLow()*/){
            aBoolean = 0; // arm keep movin up-top/bottom
            telemetry.addLine("can go down");
            bBoolean = 0; //we ain't moving
            cBoolean = 0;
            dBoolean = 0;
            armSystem.moveDown();
            telemetry.addData("ACTIVE", "armSystem down");
        } else if (aBoolean == 0 & bBoolean == 0){
            telemetry.addLine("set to stop");
            aBoolean = 0; // arm keep movin up-top/bottom
            bBoolean = 0; //we ain't moving
            cBoolean = 0;
            dBoolean = 0;
            armSystem.stop();
        }


        /*** if aBoolean - or if "y" or "a" had been clicked at some point **/

        if (aBoolean != 0) /*** set to move up or down to topmost ***/{

            if (aBoolean == 1 && armSystem.getSensorAsAnalogInput0() > ArmSystem.LEVEL_TOP) { /** if we're below top and we can move up **/
                telemetry.addData("set to move up, below top", "moving up");
                //armSystem.getElevatorMotor().setPower(1);
                armSystem.moveUp(); // move up
            } else if (aBoolean == -1 && armSystem.getSensorAsAnalogInput0() < ArmSystem.LEVEL_INTAKE) { /** if we're above intake and we can move down **/
                telemetry.addData("set to move down, above bottom", "moving down");
                //armSystem.getElevatorMotor().setPower(-1);
                armSystem.moveDown(); // move down
            } else {
                telemetry.addData("too much", "ain't goldilocks");
                armSystem.stop();
                aBoolean = 0; // if we're in a scenario where we can't do anything, stop moving and stop the "loop"
                bBoolean = 0;
            }
        }

        if (bBoolean != 0){ // if we want to move in the up-down context
            aBoolean = 0; //not moving through there
            if (bBoolean == 1) /** moving to carousel **/{
                if (cBoolean == 1 && armSystem.getSensorAsAnalogInput0() < ArmSystem.LEVEL_CAROUSEL){
                    armSystem.moveDown();
                }
                else if (cBoolean == -1 && armSystem.getSensorAsAnalogInput0() > ArmSystem.LEVEL_CAROUSEL){
                    armSystem.moveUp();
                }
                else{
                    armSystem.stop();
                    bBoolean = 0;
                    //cBoolean = 0;
                }
            }
            else if (bBoolean == -1){
                if (dBoolean == 1 && armSystem.getSensorAsAnalogInput0() < ArmSystem.LEVEL_BOTTOM) {
                    armSystem.moveDown();
                }
                else if (dBoolean == -1 && armSystem.getSensorAsAnalogInput0() > ArmSystem.LEVEL_BOTTOM) {
                    armSystem.moveUp();
                }
                else{
                    armSystem.stop();
                    bBoolean = 0;
                    //dBoolean = 0;
                }
            }

        }

        /*** TURNTABLE COMPONENTS - WORKING WELL **/

        if (gamepad2.dpad_right){
            turnTableSystem.moveToPosition(TurnTableSystem.LEVEL_0);
            telemetry.addData("ACTIVE", "turnTableSystem right");
        } else if (gamepad2.dpad_left){
            turnTableSystem.moveToPosition(TurnTableSystem.LEVEL_90);
            telemetry.addData("ACTIVE", "turnTableSystem left");
        } else {

        }

        /*** INTAKE SYSTEM - WORKING FINE **/

        if (gamepad2.right_bumper) {
            intakeSystem.take_in();
            telemetry.addData("ACTIVE", "intake");
        } else if (gamepad2.left_bumper) {
            intakeSystem.spit_out();
            telemetry.addData("ACTIVE", "outtake");
        } else if (gamepad2.right_stick_button) {
            intakeSystem.Carousel(teamState);
            telemetry.addData("ACTIVE", "carousel");
        } else {
            intakeSystem.setIdle();
        }

        /*** DRIVESYSTEM + TELEMETRY - WORKING FINE ***/
        driveSystem.drive(rx, -lx, ly);
        telemetry.addData("rx", rx);
        telemetry.addData("lx", lx);
        telemetry.addData("ly", ly);
        telemetry.addData("TIME_ELAPSED_MILSEC", elapsedTime.milliseconds());
        telemetry.update();
    }

}