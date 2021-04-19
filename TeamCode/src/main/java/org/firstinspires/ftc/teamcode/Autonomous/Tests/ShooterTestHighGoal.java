package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


/**
 * Shooter Testing for high goal
 *
 * @author  Nikhil
 * @version 1.0
 * @since   2020-October-26
 *
 */
@Autonomous(name= "Shooter Test High Goal ", group="Tests: ")
public class ShooterTestHighGoal extends LinearOpMode {

    DcMotorEx shooterMotor;


    @Override
    public void runOpMode() {
        shooterMotor = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMotor");
        telemetry.addData("Status", "Init Complete, Ready to Start");
        telemetry.update();
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(57, 0, 0, 15.4);

        waitForStart();
            while(!isStopRequested()){
                shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooterMotor.setTargetPosition(1000);
                shooterMotor.setVelocity(1675);
            }


    }
}
