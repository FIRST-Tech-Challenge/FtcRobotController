package teamCode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.BooleanSupplier;

import teamCode.autoSubsystems.AutoDriveSubsystem;
import teamCode.commands.ArmPositionCloseSampleCommand;
import teamCode.commands.ArmPositionHighBasketCommand;
import teamCode.commands.ArmPositionHighChamberCommand;
import teamCode.commands.ArmPositionHomeCommand;
import teamCode.commands.AscentArmCommand;
import teamCode.commands.IntakePivotCommand;
import teamCode.commands.ScoreSpecimenCommand;
import teamCode.subsystems.AscentArmSubsystem;
import teamCode.subsystems.IntakePivotSubsystem;
import teamCode.subsystems.IntakeWheelSubsystem;
import teamCode.subsystems.LiftArmSubsystem;
import teamCode.subsystems.SlideArmSubsystem;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoDoubleSpecimen")
public class AutoDoubleSpecimen extends LinearOpMode
{
    private DcMotor m_fLMotor;
    private DcMotor m_fRMotor;
    private DcMotor m_bLMotor;
    private DcMotor m_bRMotor;
    private DcMotor m_liftArmMotor;
    private DcMotor m_slideArmMotor;
    private CRServo m_intakeWheelServo;
    private IntakePivotSubsystem m_intakePivotSubsystem;
    private AscentArmSubsystem m_ascentArmSubsystem;
    private AutoDriveSubsystem m_autoDriveSubsystem;
    private LiftArmSubsystem m_liftArmSubsystem;
    private SlideArmSubsystem m_slideArmSubsystem;
    private IntakeWheelSubsystem m_intakeWheelSubsystem;
    private ArmPositionHighBasketCommand m_armPositionHighBasketCommand;
    private ArmPositionHighChamberCommand m_armPositionHighChamberCommand;
    private ArmPositionHomeCommand m_armPositionHomeCommand;
    private ArmPositionCloseSampleCommand m_armPositionCloseSampleCommand;
    private IntakePivotCommand m_intakePivotCommand;
    private AscentArmCommand m_ascentArmCommand;
   // private ArmFudgeFactorUpCommand m_armFudgeFactorUpCommand;
    private ScoreSpecimenCommand m_scoreSpecimenCommand;


    @Override
    public void runOpMode()
    {
        Logic.OpModeType.opMode = "Auto Double Specimen";
        this.m_fLMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        this.m_fRMotor = hardwareMap.get(DcMotor.class, "frontRight");
        this.m_bLMotor = hardwareMap.get(DcMotor.class, "backLeft");
        this.m_bRMotor = hardwareMap.get(DcMotor.class, "backRight");

        this.m_fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m_fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m_bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m_bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.m_fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.m_fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.m_bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.m_bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.m_fLMotor.setDirection(DcMotor.Direction.REVERSE);
        this.m_bLMotor.setDirection(DcMotor.Direction.REVERSE);

        this.m_fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.m_fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.m_bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.m_bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.m_liftArmMotor = hardwareMap.get(DcMotor.class, "liftArmMotor");
        this.m_slideArmMotor = hardwareMap.get(DcMotor.class, "slideArmMotor");
        this.m_intakeWheelServo = new CRServo(hardwareMap, "intakeWheelServo");
        this.m_intakePivotSubsystem = new IntakePivotSubsystem(hardwareMap, "intakePivotServo");
        this.m_ascentArmSubsystem = new AscentArmSubsystem(hardwareMap, "ascentArmServo");

        this.m_autoDriveSubsystem = new AutoDriveSubsystem(this.m_fLMotor, this.m_fRMotor, this.m_bLMotor, this.m_bRMotor);
        this.m_liftArmSubsystem = new LiftArmSubsystem(this.m_liftArmMotor);
        this.m_slideArmSubsystem = new SlideArmSubsystem(this.m_slideArmMotor);
        this.m_intakeWheelSubsystem = new IntakeWheelSubsystem(this.m_intakeWheelServo);

        this.m_armPositionHomeCommand = new ArmPositionHomeCommand(this.m_liftArmSubsystem, this.m_slideArmSubsystem);
        this.m_armPositionCloseSampleCommand = new ArmPositionCloseSampleCommand(this.m_liftArmSubsystem, this.m_slideArmSubsystem);
        this.m_armPositionHighBasketCommand = new ArmPositionHighBasketCommand(this.m_liftArmSubsystem, this.m_slideArmSubsystem);
        this.m_armPositionHighChamberCommand = new ArmPositionHighChamberCommand(this.m_liftArmSubsystem,this.m_slideArmSubsystem);
        this.m_intakePivotCommand = new IntakePivotCommand(this.m_intakePivotSubsystem);
        this.m_ascentArmCommand = new AscentArmCommand(this.m_ascentArmSubsystem);
        //this.m_armFudgeFactorUpCommand = new ArmFudgeFactorUpCommand(this.m_liftArmSubsystem);
        this.m_scoreSpecimenCommand = new ScoreSpecimenCommand(this.m_liftArmSubsystem);

        //Initialize
        this.m_armPositionHomeCommand.execute();
        this.m_intakePivotSubsystem.pivotIntake(0.85);
        waitForStart();

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-450, 450, 450, -450);//Strafe left
        wait(()-> this.m_autoDriveSubsystem.atTarget(450));

        this.m_armPositionHighChamberCommand.execute();
        wait(()-> this.m_liftArmSubsystem.atTarget(1330) && this.m_slideArmSubsystem.atTarget(-435));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(1400, 1400, 1400, 1400);//Drive forward
        wait(()-> this.m_autoDriveSubsystem.atTarget(1400));

        this.m_scoreSpecimenCommand.execute();
        sleep(1000);

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-500, -500, -500, -500);//Backup
        wait(()-> this.m_autoDriveSubsystem.atTarget(500));

        this.m_intakeWheelSubsystem.spinIntake(0.5);//Spit out
        sleep(1000);
        this.m_intakeWheelSubsystem.spinIntake(0.0);

        this.m_armPositionHomeCommand.execute();
        wait(()-> this.m_liftArmSubsystem.atTarget(0) && this.m_slideArmSubsystem.atTarget(-25));
        this.m_intakePivotSubsystem.pivotIntake(0.5);//Pivot intake

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(1025, -1025, 1025, -1025);//90 degree Right turn
        wait(()-> this.m_autoDriveSubsystem.atTarget(1025));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(1100, -1100, -1100, 1100);//Strafe right to Wallf
        wait(()-> this.m_autoDriveSubsystem.atTarget(1100));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(1000, 1000, 1000, 1000);//Drive forward
        wait(()-> this.m_autoDriveSubsystem.atTarget(1000));

        this.m_armPositionCloseSampleCommand.execute();
        wait(()-> this.m_liftArmSubsystem.atTarget(129) && this.m_slideArmSubsystem.atTarget(-812));

        this.m_intakeWheelSubsystem.spinIntake(-0.5);//Pick up Specimen

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(500, 500, 500, 500);//Drive forward
        wait(()-> this.m_autoDriveSubsystem.atTarget(500));

        sleep(2000);
        this.m_intakeWheelSubsystem.spinIntake(0.0);//stop intake
        sleep(500);

        this.m_intakePivotSubsystem.pivotIntake(0.85);
        sleep(1000);

        this.m_armPositionHomeCommand.execute();
        wait(()-> this.m_liftArmSubsystem.atTarget(0) && this.m_slideArmSubsystem.atTarget(-25));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-500, 500, 500, -500);//Strafe left
        wait(()-> this.m_autoDriveSubsystem.atTarget(500));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-1025, 1025, -1025, 1025);//90 degree Left turn
        wait(()-> this.m_autoDriveSubsystem.atTarget(1025));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-1900, 1900, 1900, -1900);//Strafe left
        wait(()-> this.m_autoDriveSubsystem.atTarget(1900));

        this.m_armPositionHighChamberCommand.execute();
        wait(()-> this.m_liftArmSubsystem.atTarget(1330) && this.m_slideArmSubsystem.atTarget(-435));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(900, 900, 900, 900);//Drive forward
        wait(()-> this.m_autoDriveSubsystem.atTarget(900));

        this.m_scoreSpecimenCommand.execute();
        sleep(1000);

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-500, -500, -500, -500);//Backup
        wait(()-> this.m_autoDriveSubsystem.atTarget(500));

        this.m_intakePivotSubsystem.pivotIntake(0.5);//Pivot intake
        this.m_armPositionHomeCommand.execute();
        wait(()-> this.m_liftArmSubsystem.atTarget(0) && this.m_slideArmSubsystem.atTarget(-25));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(2400, -2400, -2400, 2400);//Strafe right to Wallf
        wait(()-> this.m_autoDriveSubsystem.atTarget(2400));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-700, -700, -700, -700);//Backup
        wait(()-> this.m_autoDriveSubsystem.atTarget(700));
        sleep(2000);
    }

    /**
     * @Params: Wait until parameter event is true.
     */
    public void wait(@NonNull BooleanSupplier condition)
    {
        while(!condition.getAsBoolean() && opModeIsActive())
        {
            telemetry.addData("Waiting: ", condition.getAsBoolean());
            telemetry.update();
        }
    }
}