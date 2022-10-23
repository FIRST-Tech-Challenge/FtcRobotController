package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;

@TeleOp(name="Robot: Test Scaffold", group="Robot")
public class TestTeleOp extends LinearOpMode {
    public BrainStemRobot robot;
    public final double SLOW = 1.0;
    public HashMap stateMap;

    public void runOpMode(){

        robot = new BrainStemRobot(hardwareMap, telemetry, stateMap);
        //Turret turret = new Turret(hardwareMap, telemetry);
        //robot.initializeRobotPosition();
        Lift lift = new Lift(hardwareMap, telemetry, this);


        // Preset positions for turret via buttons x, y, b

        /*
        if(gamepad1.b){
            robot.turret.moveToPreset(turret.TURRET_RIGHT_POSITION, robot.lift);
        }
        if(gamepad1.x){
            robot.turret.moveToPreset(turret.TURRET_CENTER_POSITION, robot.lift);
        }

        // operator controlled lift via left stick y axis

         */
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                lift.setMotor((gamepad1.left_stick_y * SLOW));
            } else {
                lift.setMotor(0);
            }

            if(gamepad1.y){
                stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.POLE_ONE);
            }

            if (gamepad1.x) {
                stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.PLACEMENT_HEIGHT);
            } else {
                stateMap.put(robot.lift.LIFT_SUBHEIGHT, robot.lift.DELIVERY_HEIGHT);
            }

            robot.updateSystems();
        }

        // operator controlled turret via right stick x axis
        /*
        if(gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2){
            robot.turret.setMotor((gamepad1.right_stick_x * SLOW));
        }
        else{
            robot.turret.setMotor(0);
        }

        // operator controlled extension via right stick y axis
        if(gamepad1.right_stick_y > 0.2 || gamepad1.right_stick_y < -0.2){
            robot.arm.extend(gamepad1.right_stick_y);   // extend adjusts move speed, no need for SLOW multiplier
        }
        else{
            robot.arm.extend(0);
        }

        // TODO: The following segment will not work. Need to latch the position until next time the trigger is clicked.
        // Grab or drop the cone via right trigger (toggle)
        if (gamepad1.right_bumper) {
            if (robot.arm.grabberPosition() > robot.arm.GRABBER_POSITION_OPEN)
                robot.arm.grabberOpen();
            else
                robot.arm.grabberHold();
        }

        // Tilt the grabber via left trigger; hold to keep it tilted, release to reset tilt
        if (gamepad1.left_bumper) {
            robot.arm.tiltUp();
        }
        else {
            robot.arm.tiltDown();
        }

         */
        telemetry.addData("Lift" , lift.getPosition());
        //telemetry.addData("Turret", robot.turret.getPosition());
        //telemetry.addData("Extension", robot.arm.getExtensionPosition());

        telemetry.update();


    }

}
