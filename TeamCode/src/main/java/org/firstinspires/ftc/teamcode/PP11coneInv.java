//package org.firstinspires.ftc.teamcode;
//
//import static org.inventors.ftc.robotbase.RobotEx.OpModeType.AUTO;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
////import org.firstinspires.ftc.teamcode.myroadrunner.drive.SampleMecanumDrive;
//import org.inventors.ftc.opencvpipelines.AprilTagDetectionPipeline;
//import org.firstinspires.ftc.teamcode.powerplayV2.AprilTagDetectionSubsystem;
//import org.firstinspires.ftc.teamcode.powerplayV2.PowerPlayRobotV2;
//import org.firstinspires.ftc.teamcode.powerplayV2.RoadRunnerSubsystem;
//import org.firstinspires.ftc.teamcode.powerplayV2.commands.ElevatorCommand;
//import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.BasketSubsystem;
//import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ElevatorSubsystem;
//import org.inventors.ftc.robotbase.GamepadExEx;
//
//@Autonomous(name = "PP Inv (1 + 1 cone)", group = "Final Autonomous")
//public class PP11coneInv extends CommandOpMode {
//
//    PowerPlayRobotV2 robot;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//    protected ElapsedTime runtime;
//    protected SampleMecanumDrive drive;
//    protected RoadRunnerSubsystem RR;
//    protected AprilTagDetectionSubsystem april_tag;
//    protected ClawSubsystem claw;
//    protected ElevatorSubsystem elevator;
//    protected BasketSubsystem basket;
//    protected ArmSubsystem arm;
//
//    @Override
//    public void initialize() {
//        GamepadExEx driverOp = new GamepadExEx(gamepad1);
//        GamepadExEx toolOp = new GamepadExEx(gamepad2);
//
//        robot = new PowerPlayRobotV2(hardwareMap, telemetry, driverOp, toolOp, AUTO, true,
//                false, false, false, false, false);
//
//        drive = new SampleMecanumDrive(hardwareMap);
//
//        RR = new RoadRunnerSubsystem(drive, hardwareMap, true);
//
//        april_tag = new AprilTagDetectionSubsystem(robot.camera, telemetry);
//
//        claw = new ClawSubsystem(hardwareMap);
//        elevator = new ElevatorSubsystem(hardwareMap);
//        basket =  new BasketSubsystem(hardwareMap);
//        arm = new ArmSubsystem(hardwareMap, telemetry);
//
//        runtime = new ElapsedTime();
//    }
//
//    public void waitForStart() {
//        /////////////////////////////////// Recognizing the Tag ///////////////////////////////////
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         */
//        while (!isStarted() && !isStopRequested()) {
//            april_tag.aprilTagCheck();
//            sleep(20);
//        }
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        // TODO: Make telemetry subsystem/command and remove this function
//        robot.telemetryUpdate();
//        robot.dashboardTelemetryUpdate();
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initialize();
//        waitForStart();
//        runtime.reset();
//
//        ///////////////////////////////// Running the Trajectories /////////////////////////////////
//
//        int i = 0;
//
//        if (isStopRequested()) return;
//
//        RR.runHS();
//
//        schedule(new SequentialCommandGroup(
//                new InstantCommand(arm::setMid, arm),
//                new WaitCommand(300),
//                new ElevatorCommand(elevator, ElevatorSubsystem.Level.HIGH),
//                new InstantCommand(basket::setOuttake, basket),
//                new WaitCommand(1500),
//                new ParallelCommandGroup(
//                        new InstantCommand(basket::setTravel, basket),
//                        new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW)
//                )
//        ));
//
//        if (april_tag.getTagOfInterest().id == april_tag.LEFT) RR.runP1();
//        else if (april_tag.getTagOfInterest().id == april_tag.RIGHT|| april_tag.getTagOfInterest() == null) RR.runP3();
//        else RR.runTOMID();
//
//        // run the scheduler
//        while (!isStopRequested() && opModeIsActive()) {
//            run();
//        }
//        reset();
//    }
//}