package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.commands.DriveDistanceCmd;
import org.firstinspires.ftc.teamcode.commands.TurnCmd;
import org.firstinspires.ftc.teamcode.commands.ArmMed;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;
import org.firstinspires.ftc.teamcode.subsystems.ImuSub;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;

public class AutoCommandOpMode extends CommandOpMode
{
    private double turnSpeed = 0.4;
    private double driveSpeed = 0.5;

    private DrivetrainSub drive;
    private ImuSub imu;
    //private WebcamSub webcam;
    //protected IntakeSub intake;

    //protected ArmDistanceCmd armDown;
    //protected ArmDistanceCmd armNeutral;
    //protected ArmDistanceCmd armUp;

    //private VisionPortal teamPropVisionPortal;

    public ArmSub arm;
    public ArmMed armMed;

    private boolean fieldCentric = true;
    @Override
    public void initialize() {
        //Initializing Hardware
        drive = new DrivetrainSub(hardwareMap, telemetry);
        imu = new ImuSub(hardwareMap, telemetry);
        //webcam = new WebcamSub(hardwareMap, telemetry);
        //intake = new IntakeSub(hardwareMap, telemetry);
        arm = new ArmSub(hardwareMap, telemetry);
        armMed = new ArmMed(arm, drive);

//        arm.resetEncoder();
//
//        armDown = new ArmDistanceCmd(arm,telemetry,-0.5,1000);
//        armNeutral = new ArmDistanceCmd(arm,telemetry,0.5,850);
//        armUp = new ArmDistanceCmd(arm,telemetry,0.5,0);
//        teamPropVisionProcessor = new TeamPropVisionProcessor();
//        teamPropVisionPortal = VisionPortal.easyCreateWithDefaults(webcam.getWebcamName(), teamPropVisionProcessor);
//
//        //Find the position of team goal
//        TeamPropVisionProcessor.Selected branch = TeamPropVisionProcessor.Selected.NONE;

        while(opModeInInit()){
            //branch = teamPropVisionProcessor.getSelection();
            //telemetry.addData("Branch = ", branch.toString());
            telemetry.update();
        }

        //teamPropVisionPortal.close();

        waitForStart();

        //Set up vision processor
        //AprilTagVisionPortal aprilTagVisionPortal = new AprilTagVisionPortal(webcam.getWebcamName(), telemetry);
        //aprilTagVisionPortal.initialize();

        logic();

//        schedule(new SequentialCommandGroup(new InstantCommand(() -> {
//            aprilTagVisionPortal.close();
//        })));
    }

    public void logic(){}

    public TurnCmd turnCW(int angle){
        return new TurnCmd(-angle,turnSpeed,drive,imu,telemetry);
    }

    public TurnCmd turnCCW(int angle){
        return new TurnCmd(angle,turnSpeed,drive,imu,telemetry);
    }

    public DriveDistanceCmd drive(int inches){
        return new DriveDistanceCmd(inches, driveSpeed, drive, telemetry);
    }
}