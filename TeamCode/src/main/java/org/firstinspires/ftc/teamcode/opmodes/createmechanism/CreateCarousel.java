package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.carousel.MoveCarousel;
import org.firstinspires.ftc.teamcode.commands.carousel.MoveCarouselToPosition;
import org.firstinspires.ftc.teamcode.commands.carousel.StopCarousel;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;

import java.util.function.BooleanSupplier;

public class CreateCarousel {

    private CarouselSubsystem carousel;
    private final HardwareMap hwMap;
    private final String deviceName;
    private final Telemetry telemetry;
    private GamepadEx op;

    private static final double MOVE_RIGHT_POWER = 0.8;
    private static final double MOVE_LEFT_POWER = -0.8;
    private static final double MOVE_AUTO_POWER = 0.8;

    private static final double SPINNER_WHEEL_CIRC = 12.567; //inches
    private static final double CAROUSEL_WHEEL_CIRC = 47.124; //inches
    private static final double GEAR_RATIO = 3.0 / 4.0;
    private static final double MOTOR_ENCODER_COUNT = 1120 * GEAR_RATIO; //840
    private static final int CAROUSEL_MAX_ENCODER_COUNT = (int)( CAROUSEL_WHEEL_CIRC / SPINNER_WHEEL_CIRC * MOTOR_ENCODER_COUNT + 1300);

    private MoveCarousel moveCarouselRight;
    private MoveCarousel moveCarouselLeft;
    private MoveCarouselToPosition moveCarouselToPosition;
    private StopCarousel stopCarousel;

    public CreateCarousel (final HardwareMap hwMap, final String deviceName, final GamepadEx op, Telemetry telemetry){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.op = op;
        this.telemetry = telemetry;

    }

    public CreateCarousel (final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.telemetry = telemetry;

    }

    public CreateCarousel (final HardwareMap hwMap, final String deviceName, final GamepadEx op, Telemetry telemetry, boolean autoCreate){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.op = op;
        this.telemetry = telemetry;

        if (autoCreate) create();

    }

    public void create(){

        carousel = new CarouselSubsystem(hwMap,deviceName, telemetry);
        moveCarouselRight = createMoveCarousel(MOVE_RIGHT_POWER);
        moveCarouselLeft = createMoveCarousel(MOVE_LEFT_POWER);
        stopCarousel = new StopCarousel(carousel, telemetry);

        Button carouselRight = new GamepadButton(op, GamepadKeys.Button.RIGHT_BUMPER);
        Button carouselLeft = new GamepadButton(op, GamepadKeys.Button.LEFT_BUMPER);

        carouselRight.whileHeld(moveCarouselRight);
        carouselLeft.whileHeld(moveCarouselLeft);
        carousel.setDefaultCommand(new PerpetualCommand(stopCarousel));

    }

    public void createAuto(){

        carousel = new CarouselSubsystem(hwMap,deviceName, telemetry);
        moveCarouselToPosition = createMoveCarouselToPostion();
        stopCarousel = createStopCarousel();

    }

    private MoveCarousel createMoveCarousel(double power){
        return new MoveCarousel(carousel,power, telemetry);
    }

    private MoveCarouselToPosition createMoveCarouselToPostion(){

        int maxEncoderCount = CAROUSEL_MAX_ENCODER_COUNT;
        if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED)
        {
            maxEncoderCount = -CAROUSEL_MAX_ENCODER_COUNT;
            //telemetry.addLine("redCarousel");
            //telemetry.update();
        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE){
            maxEncoderCount = -CAROUSEL_MAX_ENCODER_COUNT;
            //telemetry.addLine("blueCarousel");
            //telemetry.update();
        }

        //telemetry.addData("MoveCarouselToPosition", maxEncoderCount);
        //telemetry.addData("carousel pos", carousel.getCarouselCurrentPosition());
        //telemetry.update();
        return new MoveCarouselToPosition(carousel,maxEncoderCount,MOVE_AUTO_POWER, telemetry);
    }

    private StopCarousel createStopCarousel(){
        return new StopCarousel(carousel, telemetry);
    }

    public MoveCarouselToPosition getMoveCarouselToPosition(){
        return moveCarouselToPosition;
    }

    public StopCarousel getStopCarousel(){
        return stopCarousel;
    }

    public boolean hasMetMaxEncoderCount(){
        return Math.abs(carousel.getCarouselCurrentPosition()) >= CAROUSEL_MAX_ENCODER_COUNT;
    }

    public BooleanSupplier hasMaxEncoderCountSupplier(){
        BooleanSupplier supplier = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return hasMetMaxEncoderCount();
            }
        };

        return supplier;
    }
}
