package org.firstinspires.ftc.teamcode.Util.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.Names;
import org.firstinspires.ftc.teamcode.Util.PDFLController;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class DoubleMotorLift implements Subsystem {


    private static DcMotorEx liftLeft;
    private static DcMotorEx liftRight;

    public static int liftTarget = 0;

    public static int P = 0, D = 0, F = 0, L = 0;

    public static Names.EncoderReadingFrom readingFrom = Names.EncoderReadingFrom.BOTH ;

    static PDFLController pdflController;
    public static double power;

    public static int allowedError = 1;

    private static double currentPosition = 0;

    public static boolean enableController = true;

    public static boolean isAtTarget = false;

    public static Telemetry telemetry;
    private static final int maxCurrentLimit = 5000;



    private DoubleMotorLift() {}




    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        liftLeft = hwmap.get(DcMotorEx.class, Names.SLIDE_MOTOR_LEFT);
        liftRight = hwmap.get(DcMotorEx.class, Names.SLIDE_MOTOR_RIGHT);
        liftLeft.setDirection(Names.SLIDE_MOTOR_LEFT_DIRECTION);
        liftRight.setDirection(Names.SLIDE_MOTOR_RIGHT_DIRECTION);
        resetMotors();

        telemetry = new MultipleTelemetry(opMode.getOpMode().telemetry, FtcDashboard.getInstance().getTelemetry());
        setDefaultCommand(runController());
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
    }


    @NonNull
    public static Lambda goToLiftTarget(int target){
        return new Lambda("Set Lift Target")
                .addRequirements(INSTANCE)
                .setInit(() -> liftTarget = target)
                .setFinish(() -> isAtTarget);
    }


    public static Lambda runController(){
        return new Lambda("Run Controller")
                .setExecute(() -> {
                    if(enableController){
                        currentPosition = getCurrentPosition();


                        pdflController.setPDFL(P, D, F, L);
                        pdflController.setTarget(liftTarget);
                        pdflController.update(currentPosition);



                        power = pdflController.runPDFL(allowedError);
                        liftLeft.setPower(power);
                        liftRight.setPower(power);

                        isAtTarget = Math.abs(liftTarget - currentPosition) < allowedError;
                    }
                })
                .setFinish(() -> false);
    }

    public static Lambda home() {
        return new Lambda("home-outtake")
                .setInit(() -> {
                    enableController = false;
                    setPower(-1);
                })
                .setFinish(() -> isOverCurrent(maxCurrentLimit))
                .setEnd((interrupted) -> {
                    if (!interrupted) {
                        resetMotors();
                    }
                    setPower(0);
                    enableController = true;
                });
    }

    public static void setPower(double power) {
        enableController = false;
        liftLeft.setPower(power);
        liftRight.setPower(power);
    }

    public static void resetMotors() {
        liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void setLiftTarget(int target){
        liftTarget = target;
    }

    public static void log(Names.loggingState state){
        switch (state){

            case DISABLED:
                break;
            case ENABLED:
                telemetry.addData("Target ", liftTarget);
                telemetry.addData("Current Read Position ", getCurrentPosition());
                telemetry.addData("Is At Position ", isAtTarget);
                break;
            case EXTREME:
                telemetry.addData("Power Supplied ", power);
                telemetry.addData("Left Encoder ", liftLeft.getCurrentPosition());
                telemetry.addData("Right Encoder ", liftRight.getCurrentPosition());
                telemetry.addData("Target ", liftTarget);
                telemetry.addData("Reading From ", readingFrom);
                telemetry.addData("Current Left (MA) ", liftLeft.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Current Right (MA) ", liftRight.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Current Read Position ", getCurrentPosition());
                telemetry.addData("Is At Position ", isAtTarget);
                break;



        }

    }

    public static boolean isOverCurrent(double limit) {
        return liftRight.getCurrent(CurrentUnit.MILLIAMPS) > limit || liftLeft.getCurrent(CurrentUnit.MILLIAMPS) > limit;
    }


    public static int getCurrentPosition(){
        return readingFrom == Names.EncoderReadingFrom.LEFT ? liftLeft.getCurrentPosition()   :
                (int) (readingFrom == Names.EncoderReadingFrom.RIGHT ? liftRight.getCurrentPosition() :
                        (double) (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / 2);
    }

    public static void setController(boolean state){
        enableController = state;
    }



    //Everything down here can be copy and pasted
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

    private static final DoubleMotorLift INSTANCE = new DoubleMotorLift();
}
