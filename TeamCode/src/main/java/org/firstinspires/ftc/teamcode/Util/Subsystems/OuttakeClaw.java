package org.firstinspires.ftc.teamcode.Util.Subsystems;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.Names;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.util.StateMachine;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class OuttakeClaw implements Subsystem {
    public static Servo clawServo;
    public static Servo sideServo;
    public static final OuttakeClaw INSTANCE = new OuttakeClaw();

    private OuttakeClaw() { }

    private static final StateMachine<ClawStates> clawStates = new StateMachine<>(ClawStates.CLOSED)
            .withState(ClawStates.CLOSED, (stateRef, name) -> closeClaw())
            .withState(ClawStates.OPEN, (stateRef, name) -> openClaw());

    private static final StateMachine<SideStates> sideStates = new StateMachine<>(SideStates.PARA)
            .withState(SideStates.PERP, (stateRef, name) -> perpClaw())
            .withState(SideStates.PARA, (stateRef, name) -> paraClaw());


    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        clawServo = hwmap.get(Servo.class, "intakeClaw");
        clawServo.setDirection(Servo.Direction.REVERSE);

        sideServo = hwmap.get(Servo.class, "intakeSide");
    }

    //Started
    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){
        clawServo.setPosition(.25); //Middle position
    }



    private static void close(){ clawServo.setPosition(Names.INTAKE_CLAW_CLOSED); clawStates.setState(ClawStates.CLOSED);}
    private static void open(){clawServo.setPosition(Names.INTAKE_CLAW_OPEN); clawStates.setState(ClawStates.OPEN); }

    private static void perp(){sideServo.setPosition(Names.INTAKE_SIDE_PERP); sideStates.setState(SideStates.PERP);}
    private static void para(){sideServo.setPosition(Names.INTAKE_SIDE_PARA); sideStates.setState(SideStates.PARA);}

    @NonNull
    public static Lambda toggleClaw(){
        return new Lambda("outtake claw toggle")
                .setInit(() -> {
                    switch (clawStates.getState()){
                        case OPEN:
                            clawStates.schedule(ClawStates.CLOSED);
                            break;
                        case CLOSED:
                            clawStates.schedule(ClawStates.OPEN);
                            break;
                    }
                })
                .setFinish(() -> true);


    }
    @NonNull
    public static Lambda closeClaw(){
        return new Lambda("Outtake Close Claw")
                .addRequirements(INSTANCE)
                .setInit(OuttakeClaw::close);
    }

    @NonNull
    public static Lambda openClaw(){
        return new Lambda("Outtake Open Claw")
                .addRequirements(INSTANCE)
                .setInit(OuttakeClaw::open);
    }


    @NonNull
    public static Lambda perpClaw(){
        return new Lambda("Outtake Perp Claw")
                .addRequirements(INSTANCE)
                .setInit(OuttakeClaw::perp);
    }

    @NonNull
    public static Lambda paraClaw(){
        return new Lambda("Outtake Para Claw")
                .addRequirements(INSTANCE)
                .setInit(OuttakeClaw::para);
    }






    private enum ClawStates {
        OPEN,
        CLOSED
    }
    private enum DownStates {
        DOWN,
        UP
    }
    private enum SideStates {
        PERP,
        PARA
    }

    //Everything down here can be copy and pasted
    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
}