package pedroPathing.tuners_tests.automatic;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 * You can adjust the max velocity the robot will hit on FTC Dashboard: 192/168/43/1:8080/dash
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@Autonomous(name = "Forward Zero Power Acceleration Tuner", group = "Automatic Tuners")
public class ForwardZeroPowerAccelerationTuner extends OpMode {
    private ArrayList<Double> accelerations = new ArrayList<>();

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private PoseUpdater poseUpdater;

    public static double VELOCITY = 30;

    private double previousVelocity;

    private long previousTimeNano;

    private Telemetry telemetryA;

    private boolean stopping;
    private boolean end;

    /**
     * This initializes the drive motors as well as the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        poseUpdater = new PoseUpdater(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("The robot will run forward until it reaches " + VELOCITY + " inches per second.");
        telemetryA.addLine("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetryA.addLine("Make sure you have enough room.");
        telemetryA.addLine("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.");
        telemetryA.addLine("Press CROSS or A on game pad 1 to stop.");
        telemetryA.update();
    }

    /**
     * This starts the OpMode by setting the drive motors to run forward at full power.
     */
    @Override
    public void start() {
        leftFront.setPower(1);
        leftRear.setPower(1);
        rightFront.setPower(1);
        rightRear.setPower(1);
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing CROSS or A on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    @Override
    public void loop() {
        if (gamepad1.cross || gamepad1.a) {
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setPower(0);
            }
            requestOpModeStop();
        }

        poseUpdater.update();
        Vector heading = new Vector(1.0, poseUpdater.getPose().getHeading());
        if (!end) {
            if (!stopping) {
                if (MathFunctions.dotProduct(poseUpdater.getVelocity(), heading) > VELOCITY) {
                    previousVelocity = MathFunctions.dotProduct(poseUpdater.getVelocity(), heading);
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    for (DcMotorEx motor : motors) {
                        motor.setPower(0);
                    }
                }
            } else {
                double currentVelocity = MathFunctions.dotProduct(poseUpdater.getVelocity(), heading);
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < FollowerConstants.pathEndVelocityConstraint) {
                    end = true;
                }
            }
        } else {
            double average = 0;
            for (Double acceleration : accelerations) {
                average += acceleration;
            }
            average /= (double) accelerations.size();

            telemetryA.addData("forward zero power acceleration (deceleration):", average);
            telemetryA.update();
        }
    }
}
