/*
Observation Notes on Spark Accur
 */


package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.SparkOdo;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;


//@TeleOp(name="SparkTest", group = "Subsystem Tests")
public class SparkTest extends LinearOpMode {



    private SparkOdo odo;

    private double distance_expected = 12; //inches

    private double distance_measured = 12;//12.375; //inches

    private double SPARK_DISTANCE_MULTIPLIER = distance_expected / distance_measured;

    private Mecanum robot;
    private GamepadEvents controller1;
    @Override
    public void runOpMode() throws InterruptedException {
        //Robot Initialization
        robot = new Mecanum(hardwareMap);
        controller1 = new GamepadEvents(gamepad1);

        //Spark initialization
        odo = new SparkOdo(hardwareMap);

        telemetry.update();

        waitForStart();
        //Start
        while(!isStopRequested()){

            //Drive set up
            double forward = controller1.left_stick_y;
            double strafe = controller1.left_stick_x;
            double rotate = controller1.right_stick_x;

            robot.drive(forward, strafe, rotate);



            SparkFunOTOS.Pose2D totalDist = odo.updateTotalDist();
            SparkFunOTOS.Pose2D pos = odo.getPos();

            // Reset the tracking if the user requests it
            if (controller1.y.onPress()) {
                odo.resetOdo();
            }

            // Re-calibrate the IMU if the user requests it
            if (controller1.x.onPress()) {
                odo.calibrateOdo();
            }

//            String total ="X: " + totalDist.x + "  Y: " + totalDist.y + " R: " + totalDist.h;
//            String dist = "X: " + pos.x + "  Y: " + pos.y + " R: " + pos.h;
            telemetry.addData("Total Distance",String.format("X: %f Y: %f R:%.2f", totalDist.x, totalDist.y, Math.toDegrees(totalDist.h)));
            telemetry.addData("Distance From Start", String.format("X: %f Y: %f R:%.2f", pos.x, pos.y, pos.h));
            telemetry.addData("IMU Yaw: ", robot.getYaw());
            telemetry.update();
            controller1.update();

        }
    }
}
