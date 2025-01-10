package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.DoubleHorizontalExtendo;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.visionex.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name ="AAurabot TeleOp")
public class TeleOp extends LinearOpMode {
    private GamepadEvents controller1, controller2;
    private MechDrive robot;
    private LimelightLocalization limelight;
    private DriverHubHelp screen;
    private int liftPos;
    private Claw claw;
    private double clawPos;
    public boolean clawIn = false;
    private Lift lift;
    private int expectedManualLiftPos;
    private Climb climb;
    private boolean fieldCentric = false;
    private DoubleHorizontalExtendo extendo;
    private Bucket bucket;
    private Pivot pivot;
    public void runOpMode() throws InterruptedException{

        expectedManualLiftPos = 0;
        controller1 = new GamepadEvents(gamepad1);
        controller2 = new GamepadEvents(gamepad2);
        robot = new MechDrive(hardwareMap);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        screen = new DriverHubHelp();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap, "lift", "lift");
        extendo = new DoubleHorizontalExtendo(hardwareMap, "hExtendo", "hExtendo");
//       climb = new Climb(hardwareMap,"climb");
        bucket = new Bucket(hardwareMap, "bucket");
        pivot = new Pivot(hardwareMap, "pivot", "pivot");
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
            //bucket
            if(controller1.y.onPress())
            {
                bucket.toggle();
                telemetry.addData("Bucket Pos", bucket.getPosition());
            }

            if(controller1.x.onPress())
            {
                pivot.toggle();
            }

            if(controller1.a.onPress())
            {
                pivot.goBack();
            }

            if(controller1.left_bumper.onPress())
            {
                lift.basicToggle();
            }else if(controller1.right_bumper.onPress())
            {
                lift.lower();
            }
            // horizontal extendo
            double extendoPower = -(controller1.left_trigger.getTriggerValue() - controller1.right_trigger.getTriggerValue()) * 100;
            extendo.setPositionWithLimit(extendoPower + extendo.getPosition());
            telemetry.addData("Extendo pos: ", extendo.getPosition());

            telemetry.update();
            controller1.update();


        }


    }

}

