package org.firstinspires.ftc.teamcode.opmodes.tests.Distance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;


@Autonomous
public class ColorTest extends LinearOpMode {
    // Define a variable for our color sensor
    TurtleRobotAuto robot = new TurtleRobotAuto(this);

    ColorSensor color;
    DistanceSensor distance;

    private double ColorDistance;
    private Double StraightDistance;
    static final double FEET_PER_METER = 3.28084;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   =  3.7795276;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            Alignment();
            telemetry.addData("ColorDistance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("", "");
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("","");

            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();

        }

    }



    public void Alignment () {
        StraightDistance = distance.getDistance(DistanceUnit.INCH);
        while (StraightDistance > 9) {
            StraightDistance = distance.getDistance(DistanceUnit.INCH);
            straight(0.25, 1);
        }
        straight(0, 100);
    }

    public void straight(double power, int time) {
        robot.leftfrontmotor.setPower(power);
        robot.leftbackmotor.setPower(power);
        robot.rightfrontmotor.setPower(power);
        robot.rightbackmotor.setPower(power);
        sleep(time);
    }
}
