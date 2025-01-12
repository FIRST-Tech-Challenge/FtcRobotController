package teamCode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.BooleanSupplier;

import teamCode.autoSubsystems.AutoDriveSubsystem;

import teamCode.subsystems.SlideArmSubsystem;
import teamCode.subsystems.LiftArmSubsystem;
import teamCode.subsystems.IntakePivotSubsystem;
import teamCode.subsystems.IntakeWheelSubsystem;
import teamCode.subsystems.AscentArmSubsystem;

import teamCode.commands.ArmPositionHomeCommand;
import teamCode.commands.ArmPositionCloseSampleCommand;
import teamCode.commands.ArmPositionHighBasketCommand;
import teamCode.commands.IntakePivotCommand;
import teamCode.commands.AscentArmCommand;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoSample")
public class AutoSample extends LinearOpMode
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
    private ArmPositionHomeCommand m_armPositionHomeCommand;
    private ArmPositionCloseSampleCommand m_armPositionCloseSampleCommand;
    private IntakePivotCommand m_intakePivotCommand;
    private AscentArmCommand m_ascentArmCommand;


    @Override
    public void runOpMode()
    {
        Logic.OpModeType.opMode = "AutoSample";
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
        this.m_intakePivotCommand = new IntakePivotCommand(this.m_intakePivotSubsystem);
        this.m_ascentArmCommand = new AscentArmCommand(this.m_ascentArmSubsystem);


        //Initialize
        this.m_armPositionHomeCommand.execute();
        this.m_intakePivotSubsystem.pivotIntake(0.85);
        waitForStart();

        this.m_autoDriveSubsystem.driveRobot(760, -760, -760, 760);//Strafe right
        this.m_intakePivotSubsystem.pivotIntake(0.5);//Pivot intake
        wait(()-> this.m_autoDriveSubsystem.atTarget(760));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(640, 640, 640, 640);//Drive forward
        wait(()-> this.m_autoDriveSubsystem.atTarget(640));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-512, 512, -512, 512);//45 degree Left turn
        wait(()-> this.m_autoDriveSubsystem.atTarget(512));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(450, 450, 450, 450);//Drive forward
        wait(()-> this.m_autoDriveSubsystem.atTarget(450));

        this.m_armPositionHighBasketCommand.execute();
        wait(()-> this.m_liftArmSubsystem.atTarget(2070) && this.m_slideArmSubsystem.atTarget(-2220));

        this.m_intakeWheelSubsystem.spinIntake(0.5);//Score in high basket
        sleep(1000);
        this.m_intakeWheelSubsystem.spinIntake(0.0);

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-460, -460, -460, -460);//Backup
        wait(()-> this.m_autoDriveSubsystem.atTarget(460));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(512, -512, 512, -512);//45 degree Right turn
        wait(()-> this.m_autoDriveSubsystem.atTarget(512));

        this.m_armPositionHomeCommand.execute();
        wait(()-> this.m_liftArmSubsystem.atTarget(0) && this.m_slideArmSubsystem.atTarget(-25));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-500, -500, -500, -500);//Back up *was 645
        wait(()-> this.m_autoDriveSubsystem.atTarget(500));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(1150, -1150, -1150, 1150);//Strafe right to samples
        wait(()-> this.m_autoDriveSubsystem.atTarget(1150));

        this.m_armPositionCloseSampleCommand.execute();
        this.m_intakeWheelSubsystem.spinIntake(-1.0);//Pick up field sample

        sleep(1000);
        this.m_intakeWheelSubsystem.spinIntake(0.0);//stop intake
        sleep(500);
        this.m_armPositionHomeCommand.execute();

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-1150, 1150, 1150, -1150);// Strafe
        wait(()-> this.m_autoDriveSubsystem.atTarget(1150));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(425, 425, 425, 425);//Drive
        wait(()-> this.m_autoDriveSubsystem.atTarget(425));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-512, 512, -512, 512);//45 degree Left turn
        wait(()-> this.m_autoDriveSubsystem.atTarget(512));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(450, 450, 450, 450);//Drive forward
        wait(()-> this.m_autoDriveSubsystem.atTarget(450));

        this.m_armPositionHighBasketCommand.execute();
        wait(()-> this.m_liftArmSubsystem.atTarget(2070) && this.m_slideArmSubsystem.atTarget(-2220));

        this.m_intakeWheelSubsystem.spinIntake(0.5);//Score in high basket
        sleep(1000);
        this.m_intakeWheelSubsystem.spinIntake(0.0);

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-410, -410, -410, -410);
        wait(()-> this.m_autoDriveSubsystem.atTarget(410));

        this.m_armPositionHomeCommand.execute();
        wait(()-> this.m_liftArmSubsystem.atTarget(0) && this.m_slideArmSubsystem.atTarget(-25));
        sleep(2000);

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(512, -512, 512, -512);//45 degree Right turn
        wait(()-> this.m_autoDriveSubsystem.atTarget(512));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(2000, -2000, -2000, 2000);//Strafe right to samples
        wait(()-> this.m_autoDriveSubsystem.atTarget(2000));

        this.m_autoDriveSubsystem.stop();
        this.m_autoDriveSubsystem.driveRobot(-1100, -1100, -1100, -1100);//Back up *was 645
        wait(()-> this.m_autoDriveSubsystem.atTarget(1100));

        this.m_ascentArmCommand.execute();
        this.m_ascentArmSubsystem.ascentArm(0.78);
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