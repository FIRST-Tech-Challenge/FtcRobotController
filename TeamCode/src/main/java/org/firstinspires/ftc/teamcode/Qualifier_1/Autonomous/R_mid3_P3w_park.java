//For this autonomous program, the robot should be ALIGNED TO THE LAUNCH LINE CLOSER TO THE SIDE OF THE FIELD

package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "R_mid3_P3w_park")
public class R_mid3_P3w_park extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Chassis Motors
        DcMotor motorLeftFront;
        DcMotor motorRightFront;
        DcMotor motorLeftBack;
        DcMotor motorRightBack;

        // Chassis motors
        motorLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");

        // Chassis Motors
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder count kept by left motor.
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot robot=new Robot(this, BasicChassis.ChassisType.IMU);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        robot.moveWobbleGoalServo(false);
        robot.moveAngle(-13, -57, 0.5);
        telemetry.addData("Moving Backwards", 60);
        telemetry.update();
        robot.shootHighGoal(3);
        telemetry.addData("Shooting Rings", 3);
        telemetry.update();
        robot.moveAngle(3, 0, 0.5);
        robot.moveAngle(-30.5, -34, 0.5);
        telemetry.addData("MoveAngle", 3);
        telemetry.update();
        robot.moveBackward(7, 0.5);
        robot.moveWobbleGoalServo(true);
        sleep(1000);
        robot.moveAngle(8, 40, 0.5);
    }
}