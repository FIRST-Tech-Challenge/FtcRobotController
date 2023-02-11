package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamUtil.*;

@TeleOp(name="Worlds Drive Test", group="Worlds")
public class teleopOscarTest extends OpMode{
    robotConfig r;
    private boolean armState = true;
    private boolean g1LBumperState = false;
    private double driveThrottle = 1.0;
    private boolean limitWasPressed = false;
    private final ElapsedTime runTime = new ElapsedTime();

    @Override
    public void init() {
        r = robotConfig.getInstance(this);
        r.initSystems(
                robotConstants.configuredSystems.MECANUM,
                robotConstants.configuredSystems.ARM,
                robotConstants.configuredSystems.WRIST,
                robotConstants.configuredSystems.INTAKE,
                robotConstants.configuredSystems.LIFT
        );
    }

    @Override
    public void init_loop() {
        //vision detect loop goes here
    }

    @Override
    public void start() {
        runTime.reset();
        //set claw to halfway point goes here
    }

    @Override
    public void loop() {
        boolean armInput = gamepad1.right_bumper;
        boolean driveThrottleButton = gamepad1.left_bumper;
        boolean intakeInput = gamepad2.right_trigger>0.01;
        double rightStrafePower = gamepad1.right_trigger*0.7*driveThrottle;
        double leftStrafePower = gamepad1.left_trigger*0.7*driveThrottle;
        double rightDrivePower = gamepad1.right_stick_y*driveThrottle;
        double leftDrivePower = gamepad1.left_stick_y*driveThrottle;
        boolean limitIsPressed = r.limitSwitch.isPressed();

        if(armInput && !g1LBumperState){
            g1LBumperState = true;
            armState = !armState;
        }
        else if(!armInput && g1LBumperState){
            g1LBumperState = false;
        }

        if(!armState){
            r.arm.freeTargetPosition(0);
            r.wrist.freeTargetPosition(1.0);
        }
        else{
            r.arm.freeTargetPosition(1.0);
            r.wrist.freeTargetPosition(0);
        }

        if(intakeInput){
            r.intake.freeTargetPosition(1.0);
        }
        else {
            r.intake.freeTargetPosition(0);
        }



        if(rightStrafePower>0){
            r.mecanum.rightStrafe(rightStrafePower);
        }
        else if(leftStrafePower>0){
            r.mecanum.leftStrafe(leftStrafePower);
        }
        else{
            r.mecanum.drive(leftDrivePower, rightDrivePower);
        }

        if(driveThrottleButton){
            driveThrottle = 0.6;
        }
        else{
            driveThrottle = 1.0;
        }


        if(!limitWasPressed && limitIsPressed){
            r.lift.resetEncoder();
            limitWasPressed = true;
        }
        else if (!limitIsPressed){
            limitWasPressed = false;
        }

        update();

    }

    void update(){
        r.encoderRead.encoderBulkRead();
        r.arm.update();
        r.wrist.update();
        r.intake.update();
        r.lift.update(runTime.time(), gamepad2.right_stick_y, r.limitSwitch.isPressed(), r.lift.buttonAnalysis(gamepad2.y, gamepad2.x, gamepad2.b, gamepad2.a));
    }
}
