package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

@Autonomous(name = "extraSimpleGyroTurn", group = "Zippo")

public class extraSimpleGyroTurn extends LinearOpMode {

    testPlatformHardware robot = new testPlatformHardware();

    double time = System.currentTimeMillis();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);

        waitForStart();

        double time;

        gyroTurn(360);

        telemetry.addData("Error of -90 degree turn", gyroTurnWithErrorPercent(-90));

        time = System.currentTimeMillis();
        while (System.currentTimeMillis() - time < 3000) {

        }


        telemetry.addData("Error of 90 degree turn", gyroTurnWithErrorPercent(90));

        time = System.currentTimeMillis();
        while (System.currentTimeMillis() - time < 3000) {

        }

        telemetry.addData("Error of -225 degree turn", gyroTurnWithErrorPercent(-225));

        time = System.currentTimeMillis();
        while (System.currentTimeMillis() - time < 3000) {

        }

        telemetry.addData("Error of 225 degree turn", gyroTurnWithErrorPercent(225));
        time = System.currentTimeMillis();
        while (System.currentTimeMillis() - time < 3000) {

        }

        telemetry.update();

        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);

        while (System.currentTimeMillis() - time < 15000) {

        }
    }

    public void gyroTurn(float degrees) {
        final float startAngle = getAngle();
        robot.motorFrontLeft.setPower(Math.signum(degrees)*testPlatformHardware.TURN_SPEED);
        robot.motorBackLeft.setPower(Math.signum(degrees)*testPlatformHardware.TURN_SPEED);
        robot.motorFrontRight.setPower(-1*Math.signum(degrees)*testPlatformHardware.TURN_SPEED);
        robot.motorBackRight.setPower(-1*Math.signum(degrees)*testPlatformHardware.TURN_SPEED);
        double currentAngle;
        do {
            currentAngle = getAngle();
            telemetry.addData("Start angle",startAngle);
            telemetry.addData("Current angle",currentAngle);
            telemetry.update();
        } while (currentAngle - startAngle < degrees && robot.isBusy());
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
    }
    public String gyroTurnWithErrorPercent(float degrees) {
        final float startAngle = getAngle();
        gyroTurn(degrees);
        final float currentAngle = getAngle();
        return Double.toString(Math.signum(degrees)* 100 * ((currentAngle - startAngle) - degrees) / degrees) + "%";

    }
    public void pause(int ms) throws InterruptedException {
        try {
            wait(ms);
        } catch (InterruptedException ignored) {
        }
    }
    public float getAngle() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
