package org.firstinspires.ftc.forteaching.OpModes;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.forteaching.SwerveModule;

import java.util.List;

/*
Motors: GoBilda 5202/3/4:
0: leftRearMotor
1: leftFrontMotor
2: rightRearMotor
3: rightFrontMotor
Servos: CR Servos
0: rightRearServo
1: rightFrontServo
4: leftRearServo
5: leftFrontServo
Analog Inputs:
0: rightFrontEncoder
1: rightRearEncoder
2: leftFrontEncoder
3: leftRearEncoder
*/

@TeleOp(name = "Servo Thing")
public class SwerveServoTest extends LinearOpMode {
    SwerveModule leftFront;
    SwerveModule rightFront;
    SwerveModule leftRear;
    SwerveModule rightRear;
    private List<LynxModule> hubs = null;

    public void hwinit() {
        // Set up 'bulk updates' mode...
        if (hubs == null) {
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            leftFront = new SwerveModule(hardwareMap, "leftFront");
            rightFront = new SwerveModule(hardwareMap, "rightFront");
            leftRear = new SwerveModule(hardwareMap, "leftRear");
            rightRear = new SwerveModule(hardwareMap, "rightRear");
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // PIDFCoefficients
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(6);
        telemetry.addData("State", "Getting hardware");
        telemetry.update();
        hwinit();
        telemetry.addData("State", "Waiting for start");
        telemetry.update();
        while (!isStarted()) {
            idle();
        }
        ElapsedTime et = new ElapsedTime();
        et.reset();
        double curPower = 0;
        double dir = -.01;
        while (opModeIsActive()) {
            if (Math.abs(curPower) > .001) {
                leftFront.setRotatePower(curPower);
                leftRear.setRotatePower(curPower);
                rightFront.setRotatePower(curPower);
                rightRear.setRotatePower(curPower);
                String data = String.format("LF: %f(%f) RF: %f(%f) LR: %f(%f) RR: %f(%f)",
                        leftFront.getAngle(), curPower,
                        rightFront.getAngle(), curPower,
                        leftRear.getAngle(), curPower,
                        rightRear.getAngle(), curPower);
                telemetry.addLine(data);
                Log.d("info", data);
                telemetry.addData("Loop", et.toString());
                et.reset();
                telemetry.update();
            }
            curPower += dir;
            if (Math.abs(curPower) > .4) {
                dir = -dir;
            }
            while (et.milliseconds() < 40) {
            }
            et.reset();
        }
    }

    private static double degrees(double val) {
        while (val > 360) {
            val -= 360;
        }
        while (val < 0) {
            val += 360;
        }
        return val;
    }

}
