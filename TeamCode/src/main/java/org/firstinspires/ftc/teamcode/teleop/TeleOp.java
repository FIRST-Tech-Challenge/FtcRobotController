package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.horizontalExtendo;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name ="AAurabot TeleOp")
public class TeleOp extends LinearOpMode {
    private GamepadEvents controller1, controller2;
    private MechDrive robot;
    private Limelight limelight;
    private DriverHubHelp screen;
    private int liftPos;
    private Claw claw;
    private double clawPos;
    public boolean clawIn = false;
    private Lift lift;
    private int expectedManualLiftPos;
    private int[] liftPositions = {-1000, -4000, 1000};
    private int liftPositionIndex = 0;
    private boolean isReversing = false;
    private Climb climb;
    private boolean fieldCentric = false;
    private final double liftPower = 0.1;
    Lift manuaLift;
    private horizontalExtendo extendo;
    double headingOffsetDeg = 0;
    int testPos = 0;
    public void runOpMode() throws InterruptedException{

        expectedManualLiftPos = 0;
        controller1 = new GamepadEvents(gamepad1);
        controller2 = new GamepadEvents(gamepad2);
        robot = new MechDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        screen = new DriverHubHelp();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        claw = new Claw(hardwareMap);
//        lift = new Lift(hardwareMap, "liftLeft", "liftRight", "liftLeft", "liftRight" );
        extendo = new horizontalExtendo(hardwareMap,"hExtendo");
//       climb = new Climb(hardwareMap,"climb");
        fieldCentric = false;

        waitForStart();
        resetRuntime();

        while(opModeIsActive())
        {
            telemetry.addData("Field Centric: ", fieldCentric);

            //drive
            double forward = controller1.left_stick_y;
            double strafe = controller1.left_stick_x;
            double rotate = -controller1.right_stick_x;
            clawPos = 0.6;
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            robot.drive(forward, strafe, rotate);

            //lift

            //manual mode for lift
//            double liftTriggerValue = controller2.left_trigger.getTriggerValue() - controller2.right_trigger.getTriggerValue();
//            manuaLift.setPower(liftTriggerValue);
            telemetry.addData("Expected Target Pos", expectedManualLiftPos);

            //claw
            if (controller1.b.onPress()) {
                claw.toggle();
                telemetry.addData("Claw is Open: ", claw.getIsOpen());
            }

//            telemetry.addData("Lift Left pos", lift.getLeftPosition());
//            telemetry.addData("Lift Right pos", lift.getRightPosition());

            // horizontal extendo
            double extendoPos = (controller1.left_trigger.getTriggerValue() - controller1.right_trigger.getTriggerValue()) * 0.001;
            extendo.setPosition(extendoPos);
            telemetry.addData("Extendo pos: ", extendo.getPosition());

            telemetry.update();
            controller1.update();


        }


    }

}

