package org.firstinspires.ftc.teamcode.Drivetrain;

import androidx.annotation.NonNull;

import java.util.List;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivetrain.Controllers.DrivetrainMotorController;
import org.firstinspires.ftc.teamcode.Drivetrain.Controllers.GeometricController;
import org.firstinspires.ftc.teamcode.Drivetrain.Controllers.PoseController;
import org.firstinspires.ftc.teamcode.Drivetrain.Geometry.Path;
import org.firstinspires.ftc.teamcode.Drivetrain.Localizers.TwoWheelOdometery;
import org.firstinspires.ftc.teamcode.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Hardware.Motors.MotorController;

public class Drivetrain {
    /**
     * Hardware map
     */
    HardwareMap hardwareMap = null;
    public int opSwitch = 1;
    public SimpleMatrix state = new SimpleMatrix(6, 1);
    public TwoWheelOdometery twoWheelOdo;

    public DrivetrainMotorController motorController;
    public GeometricController geometricController;
    /**
     * Drive motors
     */
    public DcMotorEx motorLeftFront = null;
    public DcMotorEx motorLeftBack = null;
    public DcMotorEx motorRightBack = null;
    public DcMotorEx motorRightFront = null;
    public SimpleMatrix wheelPowerPrev = new SimpleMatrix(4, 1);
    public PoseController poseControl = new PoseController();
    public static double acceptablePowerDifference = 0.000001; // The acceptable difference between current and previous wheel power to make a hardware call
    public static double distanceThreshold = 0.2;
    public static double angleThreshold = 0.1;

    public SimpleMatrix prevWheelSpeeds = new SimpleMatrix( new double[][]{
            new double[]{0},
            new double[]{0},
            new double[]{0},
            new double[]{0}
    });
    public SimpleMatrix stopMatrix = new SimpleMatrix( new double[][]{
            new double[]{0},
            new double[]{0},
            new double[]{0},
            new double[]{0}
    });
    public ElapsedTime deltaT = new ElapsedTime();
    public Drivetrain(HardwareMap hwMap){
        hardwareMap = hwMap;
        motorController = new DrivetrainMotorController(hwMap);
        geometricController = new GeometricController();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "lfm");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "lbm");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "rbm");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "rfm");
        //casting

        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        /**
         * Establish that motors will not be using their native encoders:
         * 'RUN_WITHOUT_ENCODER' does not actually run without encoders, it
         * deactivates the PID
         */
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /**
         * Establish zero-power behavior
         */
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**
         * Set zero power to all motors upon initialization
         */
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        twoWheelOdo = new TwoWheelOdometery(hardwareMap);
        deltaT.reset();
    }
    public void localize() {
        state = twoWheelOdo.calculate();
    }
    public void resetOp(){
        opSwitch = 0;
    }
    public void setOp(){
        opSwitch = 1;
    }
    public void setPower(SimpleMatrix powers) {
        double u0 = powers.get(0, 0);
        double u1 = powers.get(1, 0);
        double u2 = powers.get(2, 0);
        double u3 = powers.get(3, 0);
        double u0Prev = wheelPowerPrev.get(0, 0);
        double u1Prev = wheelPowerPrev.get(1, 0);
        double u2Prev = wheelPowerPrev.get(2, 0);
        double u3Prev = wheelPowerPrev.get(3, 0);
        if (Math.abs(u0Prev - u0) > acceptablePowerDifference) {
            motorLeftFront.setPower(powers.get(0, 0));
        }
        if (Math.abs(u1Prev - u1) > acceptablePowerDifference) {
            motorLeftBack.setPower(powers.get(1, 0));
        }
        if (Math.abs(u2Prev - u2) > acceptablePowerDifference) {
            motorRightBack.setPower(powers.get(2, 0));
        }
        if (Math.abs(u3Prev - u3) > acceptablePowerDifference) {
            motorRightFront.setPower(powers.get(3, 0));
        }
        wheelPowerPrev.set(0,0,u0);
        wheelPowerPrev.set(1,0,u1);
        wheelPowerPrev.set(2,0,u2);
        wheelPowerPrev.set(3,0,u3);
    }
    public void setWheelSpeedAcceleration(SimpleMatrix wheelSpeeds, SimpleMatrix wheelAccelerations){
        setPower(motorController.calculate(wheelSpeeds,wheelAccelerations));
    }

    /*public void goToPose(SimpleMatrix desiredPose){
        SimpleMatrix pose = state.extractMatrix(0,3,0,1);
        SimpleMatrix wheelSpeeds = poseControl.calculate(pose, desiredPose);
        SimpleMatrix wheelAccelerations = wheelSpeeds.minus(prevWheelSpeeds).scale(1/deltaT.seconds());
        deltaT.reset();
        setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);
        prevWheelSpeeds = wheelSpeeds;
    }*/
    public Action goToPose(SimpleMatrix desiredPose, int step) {
        return new Action() {
            //private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //if (!initialized) {
                //    initialized = true;
                //}
                localize();
                SimpleMatrix pose = state.extractMatrix(0,3,0,1);
                SimpleMatrix wheelSpeeds = poseControl.calculate(pose, desiredPose);
//                SimpleMatrix wheelAccelerations = wheelSpeeds.minus(prevWheelSpeeds).scale(1/deltaT.seconds());
                SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);
                deltaT.reset();
                setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);
                prevWheelSpeeds = wheelSpeeds;
                if (Math.abs(Utils.calculateDistance(state.get(0,0),state.get(1,0),desiredPose.get(0,0),desiredPose.get(1,0)))<distanceThreshold&&Math.abs(Utils.angleWrap(state.get(2,0)-desiredPose.get(2,0)))<angleThreshold){
                    setPower(stopMatrix);
                    opSwitch++;
                }
                return step == opSwitch;
            }
        };
    }
    public InstantAction stopMotors() {
        return new InstantAction(()->setPower(stopMatrix));
    }
    public Action followPath(Path path) {
        return new Action() {
            // Maybe you'd be able to put an elapsedTimer here?? ****
            ElapsedTime elapsedTimer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //if (!initialized) {
                //    initialized = true;
                //}
                localize();


                // Check if the distance between the current robot position (so maybe grab the pose from the state FIRST!)
                // and the path's final waypoint is less than the geometricController's xy lookahead. If that is the case,
                // you'd want to do what?
                SimpleMatrix pose = state.extractMatrix(0,3,0,1);
                // Rename to desiredPose! It contains a heading too so furthestPoint is misleading!
                SimpleMatrix desiredPose = geometricController.calculate(state, path);
                SimpleMatrix wheelSpeeds = poseControl.calculate(pose, desiredPose);
                // not sure why you're using distanceThreshold here.
                // it would be something like path.getMotionProfile.getVelocity(t), where t is an elapsed timer.
                // ****This timer should started when you start following this path. Maybe you can put it above?
                // It shouldn't have to be reset as it should be specific to the action where you're following the path.
                double maxScale = path.getMotionProfile().getVelocity(elapsedTimer.seconds())/wheelSpeeds.elementMaxAbs(); // You also need to grab the maximum of the ABSOLUTE VALUE of all the wheel speeeds..
                wheelSpeeds.scale(maxScale);
                SimpleMatrix wheelAccelerations = new SimpleMatrix(4, 1);
                deltaT.reset(); // don't need this right?
                setWheelSpeedAcceleration(wheelSpeeds, wheelAccelerations);
                prevWheelSpeeds = wheelSpeeds;

                // Please please please, write a function in drivetrain called isClose or something, which does this checking for you!!!
                // replace it with that in both the goToPose and the followPath
                if (!(Math.abs(Utils.calculateDistance(state.get(0,0),state.get(1,0),wheelSpeeds.get(0,0),wheelSpeeds.get(1,0)))<distanceThreshold)){
                    if (Math.abs(Utils.angleWrap(state.get(2,0)-wheelSpeeds.get(2,0)))<angleThreshold) {
                        setPower(stopMatrix);
                    }
                }
                return Math.abs(Utils.calculateDistance(state.get(0,0),state.get(1,0),wheelSpeeds.get(0,0),wheelSpeeds.get(1,0)))>distanceThreshold&&Math.abs(Utils.angleWrap(state.get(2,0)-wheelSpeeds.get(2,0)))>angleThreshold;
            }
        };
    }
    public Action updateTelemetry(Telemetry telemetry) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                localize();
                telemetry.addData("x", state.get(0,0));
                telemetry.addData("y", state.get(1,0));
                telemetry.addData("theta", state.get(2,0));
                telemetry.addData("uLf", motorController.uLf);
                telemetry.addData("uLb", motorController.uLb);
                telemetry.addData("uRb", motorController.uRb);
                telemetry.addData("uRf", motorController.uRf);
                telemetry.update();
                return true;
            }
        };
    }
}
