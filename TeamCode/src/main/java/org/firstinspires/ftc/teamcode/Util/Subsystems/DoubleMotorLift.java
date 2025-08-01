package org.firstinspires.ftc.teamcode.Util.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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


    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;

    public static int liftTarget = 0;

    public static int P = 0, D = 0, F = 0, L = 0;

    public EncoderReadingFrom readingFrom = EncoderReadingFrom.LEFT ;

    PDFLController pdflController;
    public static double power;

    public static int allowedError = 1;

    private double currentPosition = 0;

    public static boolean isAtTarget = false;



    private DoubleMotorLift() {}




    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        liftLeft = hwmap.get(DcMotorEx.class, Names.SLIDE_MOTOR_LEFT);
        liftRight = hwmap.get(DcMotorEx.class, Names.SLIDE_MOTOR_RIGHT);
        liftLeft.setDirection(Names.SLIDE_MOTOR_LEFT_DIRECTION);
        liftRight.setDirection(Names.SLIDE_MOTOR_RIGHT_DIRECTION);
        resetMotors();
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
        currentPosition =
                readingFrom == EncoderReadingFrom.LEFT ? liftLeft.getCurrentPosition()   :
                        readingFrom == EncoderReadingFrom.RIGHT ? liftRight.getCurrentPosition() :
                                (double) (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / 2;


        pdflController.setPDFL(P, D, F, L);
        pdflController.setTarget(liftTarget);
        pdflController.update(currentPosition);



        power = pdflController.runPDFL(allowedError);
        liftLeft.setPower(power);
        liftRight.setPower(power);

        isAtTarget = Math.abs(liftTarget - currentPosition) < allowedError;
    }


    @NonNull
    public static Lambda goToLiftTarget(int target){
        return new Lambda("Set Lift Target")
                .addRequirements(INSTANCE)
                .setInit(() -> liftTarget = target)
                .setFinish(() -> isAtTarget);
    }




    public void resetMotors() {
        liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void setLiftTarget(int target){
        liftTarget = target;
    }


    //Everything down here can be copy and pasted
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(IntakeClaw.Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

    private static final DoubleMotorLift INSTANCE = new DoubleMotorLift();
}
