package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@Autonomous(name = "Specimen and Sample Auto")
public class SpecimenAndSample extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private Limelight limelight;
    private Imu imu;
    private TwoDeadWheelLocalizer deadwheels;
    private DriverHubHelp screen;
    double forward;
    double strafe;
    double rotate;
    private Lift lift;
    private double liftPower;
    private Arm arm;
    private double armPosition;

    private Claw claw;
    private double clawPos;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MechDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        imu = new Imu(hardwareMap);
        screen = new DriverHubHelp();
//        deadwheels = new TwoDeadWheelLocalizer(hardwareMap);
        lift = new Lift(hardwareMap, "liftLeft", "liftRight", "liftLeft", "liftRight");
        arm = new Arm(hardwareMap, "armRight", "armLeft");
        claw = new Claw(hardwareMap);
        clawPos = 1;




    }
}
