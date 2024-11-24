package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name ="Aurabot TeleOp")
public class TeleOp extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private Limelight limelight;
    private Imu imu;
//    private ThreeDeadWheelLocalizer deadwheels;
    private DriverHubHelp screen;
    GoBildaPinpointDriver odometry;
    private Arm arm;
    private double armPos;
    private Claw claw;
    private double clawPos;
    private Lift lift;
    private double liftPower;
    boolean reverseArm;
    private double[] armPositions = {0.9, 1.0,  0.3, 0};
    private int armPositionIndex = 0;
    private boolean isReversing = false;
    private Climb climb;
    private boolean fieldCentric;
    double oldTime = 0;

    double headingOffsetDeg = 0;

    public void runOpMode() throws InterruptedException{

        controller = new GamepadEvents(gamepad1);
        robot = new MechDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
//      imu = new Imu(hardwareMap);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        screen = new DriverHubHelp();
//        deadwheels = new ThreeDeadWheelLocalizer(hardwareMap);
        arm = new Arm(hardwareMap, "armRight", "armLeft");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap, "liftLeft", "liftRight","liftLeft", "liftRight");
        reverseArm = false;
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        climb = new Climb(hardwareMap,"climb");
        fieldCentric = false;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));

        imu.initialize(parameters);

        //setting up odometry
        odometry.setOffsets(-84.0, -168.0, headingOffsetDeg);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.recalibrateIMU();
        odometry.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odometry.getXOffset());
        telemetry.addData("Y offset", odometry.getYOffset());
        telemetry.addData("Device Version Number:", odometry.getDeviceVersion());
        telemetry.addData("Device Scalar", odometry.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();

        while(opModeIsActive())
        {
            if(controller.y.onPress())
            {
                fieldCentric = !fieldCentric;
            }
            if(fieldCentric)
            {
                telemetry.addData("Field Centric: ", fieldCentric);
                double forward = -controller.left_stick_y;
                double strafe = controller.left_stick_x;
                double rotate = -controller.right_stick_x;
                clawPos = 0.6;
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

                //drive and limelight
                String[] distance =limelight.getDistanceInInches();
                telemetry.addData("Limelight Distance: ", distance[0] + ", " + distance[1]);
                drive.updatePoseEstimate();
                robot.fieldCentricDrive(forward, strafe, rotate);
                telemetry.addData("x", screen.roundData(drive.pose.position.x));
                telemetry.addData("y", screen.roundData(drive.pose.position.y));
                telemetry.addData("Yaw (deg)", screen.roundData(Math.toDegrees(drive.pose.heading.toDouble())));


                //arm
                if(controller.right_bumper.onPress())
                {
                    if(!isReversing)
                    {
                        armPos = armPositions[armPositionIndex];
                        arm.setPosition(armPos);
                        armPositionIndex++;
                        if(armPositionIndex >= armPositions.length)
                        {
                            armPositionIndex--;
                            isReversing = true;
                        }
                    }else {
                        armPos = armPositions[armPositionIndex];
                        arm.setPosition(armPos);
                        armPositionIndex--;
                        if(armPositionIndex < 0)
                        {
                            armPositionIndex++;
                            isReversing = false;
                        }
                    }


                }
                if(controller.left_bumper.onPress())
                {

                    armPos = armPositions[armPositionIndex];
                    arm.setPosition(armPos);
                    armPositionIndex--;
                    if(armPositionIndex < 0)
                    {
                        armPositionIndex = armPositions.length-1;
                    }

                }
                telemetry.addData("Arm Position", armPos);


                //claw
                if (controller.a.onPress()) {
                    claw.release();
                } else if (controller.b.onPress()) {
                    claw.close(clawPos);
                }
                telemetry.addData("Claw Position", clawPos);


                //lift
                liftPower = controller.right_trigger.getTriggerValue() - controller.left_trigger.getTriggerValue();
                lift.moveLift(liftPower);
                telemetry.addData("Lift Power", liftPower);

                telemetry.update();
                controller.update();
            }
            //odometry stuff

            odometry.update();
            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

             /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odometry.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            Pose2D vel = odometry.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

             /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */
            telemetry.addData("Status", odometry.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odometry.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            //Driver controls

            telemetry.addData("Field Centric: ", fieldCentric);


            double forward = -controller.left_stick_y;
            double strafe = controller.left_stick_x;
            double rotate = -controller.right_stick_x;
            clawPos = 0.6;
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            //drive and limelight
//            String[] distance =limelight.getDistanceInInches();
//            telemetry.addData("Limelight Distance: ", distance[0] + ", " + distance[1]);
//            drive.updatePoseEstimate();
            robot.drive(forward, strafe, rotate);
//            telemetry.addData("x", screen.roundData(drive.pose.position.x));
//            telemetry.addData("y", screen.roundData(drive.pose.position.y));
//            telemetry.addData("Yaw (deg)", screen.roundData(Math.toDegrees(drive.pose.heading.toDouble())));


            //arm
            if(controller.right_bumper.onPress())
            {
                if(!isReversing)
                {
                    armPos = armPositions[armPositionIndex];
                    arm.setPosition(armPos);
                    armPositionIndex++;
                    if(armPositionIndex >= armPositions.length)
                    {
                        armPositionIndex--;
                        isReversing = true;
                    }
                }else {
                    armPos = armPositions[armPositionIndex];
                    arm.setPosition(armPos);
                    armPositionIndex--;
                    if(armPositionIndex < 0)
                    {
                        armPositionIndex++;
                        isReversing = false;
                    }
                }


            }
            if(controller.left_bumper.onPress())
            {

                armPos = armPositions[armPositionIndex];
                arm.setPosition(armPos);
                armPositionIndex--;
                if(armPositionIndex < 0)
                {
                    armPositionIndex = armPositions.length-1;
                }

            }
            telemetry.addData("Arm Position", armPos);


            //claw
            if (controller.a.onPress()) {
                claw.release();
            } else if (controller.b.onPress()) {
                claw.close(clawPos);
            }
            telemetry.addData("Claw Position", clawPos);


            //lift
            liftPower = controller.right_trigger.getTriggerValue() - controller.left_trigger.getTriggerValue();
            lift.moveLift(liftPower);
            telemetry.addData("Lift Power", liftPower);

            telemetry.update();
            controller.update();

            //Climb
            if(controller.x.onPress()) {
                liftPower = -1;
                lift.moveLift(liftPower);
                sleep(1000);
                liftPower = 0;
                lift.moveLift(liftPower);

                strafe = 0;
                rotate = 0;
                forward = 0.4;
                robot.drive(forward, strafe, rotate);
                sleep(100);

                liftPower = 1;
                lift.moveLift(liftPower);
                sleep(1000);
            }
        }


    }

}

