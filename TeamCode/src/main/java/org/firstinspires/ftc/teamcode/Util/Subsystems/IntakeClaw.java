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

public class IntakeClaw implements Subsystem {
    public static Servo clawServo;
    public static Servo downServo;
    public static Servo sideServo;
    public static final IntakeClaw INSTANCE = new IntakeClaw();

    private IntakeClaw() { }

    private static final StateMachine<ClawStates> clawStates = new StateMachine<>(ClawStates.CLOSED)
            .withState(ClawStates.CLOSED, (stateRef, name) -> closeClaw())
            .withState(ClawStates.OPEN, (stateRef, name) -> openClaw());

    private static final StateMachine<DownStates> downStates = new StateMachine<>(DownStates.UP)
            .withState(DownStates.DOWN, (stateRef, name) -> clawDown())
            .withState(DownStates.UP, (stateRef, name) -> upClaw());

    private static final StateMachine<SideStates> sideStates = new StateMachine<>(SideStates.PARA)
            .withState(SideStates.PERP, (stateRef, name) -> perpClaw())
            .withState(SideStates.PARA, (stateRef, name) -> paraClaw());


    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        clawServo = hwmap.get(Servo.class, "intakeClaw");
        clawServo.setDirection(Servo.Direction.REVERSE);
        downServo = hwmap.get(Servo.class, "intakeDown");
        sideServo = hwmap.get(Servo.class, "intakeSide");
    }

    //Started
    @Override
    public void postUserStartHook(@NonNull Wrapper opMode){
        clawServo.setPosition(.25); //Middle position
    }



    private static void close(){ clawServo.setPosition(Names.INTAKE_CLAW_CLOSED); clawStates.setState(ClawStates.CLOSED);}
    private static void open(){clawServo.setPosition(Names.INTAKE_CLAW_OPEN); clawStates.setState(ClawStates.OPEN); }

    private static void down(){downServo.setPosition(Names.INTAKE_DOWN_DOWN); downStates.setState(DownStates.DOWN);}
    private static void up(){downServo.setPosition(Names.INTAKE_DOWN_UP); downStates.setState(DownStates.UP);}

    private static void perp(){sideServo.setPosition(Names.INTAKE_SIDE_PERP); sideStates.setState(SideStates.PERP);}
    private static void para(){sideServo.setPosition(Names.INTAKE_SIDE_PARA); sideStates.setState(SideStates.PARA);}

    @NonNull
    public static Lambda toggleClaw(){
        return new Lambda("intake claw toggle")
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
        return new Lambda("Intake Close Claw")
                .addRequirements(INSTANCE)
                .setInit(IntakeClaw::close);
    }

    @NonNull
    public static Lambda openClaw(){
        return new Lambda("Intake Open Claw")
                .addRequirements(INSTANCE)
                .setInit(IntakeClaw::open);
    }

    @NonNull
    public static Lambda clawDown(){
        return new Lambda("Intake Down Claw")
                .addRequirements(INSTANCE)
                .setInit(IntakeClaw::down);
    }

    @NonNull
    public static Lambda upClaw(){
        return new Lambda("Intake Up Claw")
                .addRequirements(INSTANCE)
                .setInit(IntakeClaw::up);
    }

    @NonNull
    public static Lambda perpClaw(){
        return new Lambda("Intake Perp Claw")
                .addRequirements(INSTANCE)
                .setInit(IntakeClaw::perp);
    }

    @NonNull
    public static Lambda paraClaw(){
        return new Lambda("Intake Para Claw")
                .addRequirements(INSTANCE)
                .setInit(IntakeClaw::para);
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