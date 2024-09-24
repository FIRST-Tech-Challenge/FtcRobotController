package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control;
import org.firstinspires.ftc.teamcode.RobotClass;

import java.util.HashMap;
import java.util.Map;

@TeleOp
public class TestOpMode extends Control {
    Map<RobotClass.MOTORS, Double> wheelSpeeds = new HashMap<>();
    boolean atTarget = false;
    double angularDistance = 0;
    double targetAngle = -90.0;
    int turnVal = 1;

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();


        double turn = gamepad1.left_trigger - gamepad1.right_trigger;

        double currentAngle = Math.toDegrees(robot.getHeading());

        if(currentAngle - targetAngle < 0){
            turnVal = -1;
        }else{
            turnVal = 1;
        }

        double raw_diff = Math.abs(currentAngle - targetAngle);

        double mod_diff = raw_diff % 360.0;
        //double angularDistance = mod_diff > 180.0 ? 360.0 - mod_diff : mod_diff;
        double angularDistance = 180 - Math.abs(mod_diff - 180);



        double powerReduce = angularDistance / 90;

        powerReduce = Math.max(powerReduce, 0.2);
        powerReduce = Math.min(powerReduce, 0.9);

        wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, -turn);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, -turn);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, turn);
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, turn);

        telemetry.addData("turnVal", turnVal);
        telemetry.addData("currentAngle", currentAngle);
        telemetry.addData("raw_diff", raw_diff);
        telemetry.addData("mod_diff", mod_diff);
        telemetry.addData("angularDistance", angularDistance);
        telemetry.addData("powerReduce", powerReduce);
        telemetry.update();


        if (angularDistance < 0.5) atTarget = true;

        UpdateWheelPowers();
    }

    public void UpdateWheelPowers() {
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_LEFT));
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_RIGHT));
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_LEFT));
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_RIGHT));
    }

}
