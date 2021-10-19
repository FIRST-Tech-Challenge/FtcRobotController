package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auton", group="Pushbot")
//@Disabled
public class AutoFrenzy extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot   = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    final double countPerRev = 384.5;
    final double gearRatio = 1;
    final double wheelDi = 4.75;
    final double countPerIN = countPerRev/(wheelDi*Math.PI);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous: ", "waiting for start");
        telemetry.update();
        double distanceIN = robot.distance.getDistance(DistanceUnit.INCH);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (distanceIN > 10) {
            move(1, 'f', 100);
        }
        else{
            motorstop();
        }
    }
    public void motorstop(){
        robot.m0.set(0);
        robot.m1.set(0);
        robot.m2.set(0);
        robot.m3.set(0);
        sleep(100);
    }
    public void move(double power, char direction, long SLEEP){
        switch (direction){
            case 'b':
                robot.m0.set(power);
                robot.m1.set(power);
                robot.m2.set(power);
                robot.m3.set(power);
                sleep(SLEEP);
                break;
            case 'f':
                robot.m0.set(-power);
                robot.m1.set(-power);
                robot.m2.set(-power);
                robot.m3.set(-power);
                sleep(SLEEP);
                break;
            case 'r':
                robot.m0.set(-power);
                robot.m1.set(power);
                robot.m2.set(-power);
                robot.m3.set(power);
                sleep(SLEEP);
                break;
            case 'l':
                robot.m0.set(power);
                robot.m1.set(-power);
                robot.m2.set(power);
                robot.m3.set(-power);
                sleep(SLEEP);
                break;
            case 'x':
                robot.m0.set(1);
                robot.m1.set(.25);
                robot.m2.set(1);
                robot.m3.set(.25);
                sleep(SLEEP);
                break;
            case 'y':
                robot.m0.set(.25);
                robot.m1.set(1);
                robot.m2.set(.25);
                robot.m3.set(1);
                sleep(SLEEP);
                break;
        }
        motorstop();
    }
}