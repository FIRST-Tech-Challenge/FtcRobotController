package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.movement.api.StrafingMovement;
import org.firstinspires.ftc.teamcode.core.movement.impl.StrafedMovementImpl;

@Autonomous
public class BasicStrafedMovementTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int runTime = 1000;
        double power = 0.3;
        int restTime = 1000;

        StrafingMovement strafedMovement = new StrafedMovementImpl(
            hardwareMap.get(DcMotor.class, "front left wheel"),
            hardwareMap.get(DcMotor.class, "front right wheel"),
            hardwareMap.get(DcMotor.class, "back right wheel"),
            hardwareMap.get(DcMotor.class, "back left wheel")
        );
        waitForStart();
        telemetry.addLine("Driving forward for 1 second.");
        telemetry.update();
        strafedMovement.drive(power);
        Thread.sleep(runTime);
        strafedMovement.stop();
        Thread.sleep(restTime);


        telemetry.addLine("Driving backwards for 1 second.");
        telemetry.update();
        strafedMovement.drive(-power);
        Thread.sleep(runTime);
        strafedMovement.stop();
        Thread.sleep(restTime);

        telemetry.addLine("Rotating left for 1 second.");
        telemetry.update();
        strafedMovement.rotate(power);
        Thread.sleep(runTime);
        strafedMovement.stop();
        Thread.sleep(restTime);

        telemetry.addLine("Rotating right for 1 second.");
        telemetry.update();
        strafedMovement.rotate(-power);
        Thread.sleep(runTime);
        strafedMovement.stop();
        Thread.sleep(restTime);

        telemetry.addLine( "Strafing left for 1 second.");
        telemetry.update();
        strafedMovement.strafe(power);
        Thread.sleep(runTime);
        strafedMovement.stop();
        Thread.sleep(restTime);

        telemetry.addLine("Strafing right for 1 second.");
        telemetry.update();
        strafedMovement.strafe(-power);
        Thread.sleep(runTime);
        strafedMovement.stop();
        Thread.sleep(restTime);
    }
}
