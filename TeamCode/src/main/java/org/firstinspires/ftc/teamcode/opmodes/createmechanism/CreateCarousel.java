package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.carousel.MoveCarousel;
import org.firstinspires.ftc.teamcode.commands.carousel.StopCarousel;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;

public class CreateCarousel {

    private CarouselSubsystem carousel;
    private final HardwareMap hwMap;
    private final String deviceName;
    private final Telemetry telemetry;
    private GamepadEx op;

    private static final double MOVE_RIGHT_POWER = 0.5;
    private static final double MOVE_LEFT_POWER = -0.5;
    private static final double MOVE_AUTO_POWER = 0.3;

    private static final int MAX_ENCODER_COUNT = 1000;

    private MoveCarousel moveCarouselRight;
    private MoveCarousel moveCarouselLeft;
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

        carousel = new CarouselSubsystem(hwMap,deviceName);

        moveCarouselRight = new MoveCarousel(carousel,MOVE_RIGHT_POWER, telemetry);
        moveCarouselLeft = new MoveCarousel(carousel, MOVE_LEFT_POWER, telemetry);
        stopCarousel = new StopCarousel(carousel, telemetry);

        Button carouselRight = new GamepadButton(op, GamepadKeys.Button.RIGHT_BUMPER);
        Button carouselLeft = new GamepadButton(op, GamepadKeys.Button.LEFT_BUMPER);


        carouselRight.whileHeld(moveCarouselRight);
        carouselLeft.whileHeld(moveCarouselLeft);
        carousel.setDefaultCommand(new PerpetualCommand(stopCarousel));

    }

    public void createAuto(){

        carousel = new CarouselSubsystem(hwMap,deviceName);

        moveCarouselRight = new MoveCarousel(carousel,MOVE_AUTO_POWER, telemetry);
        stopCarousel = new StopCarousel(carousel, telemetry);

    }

    private MoveCarousel createMoveCarousel(double power){
        return new MoveCarousel(carousel,power, telemetry);
    }

    private StopCarousel createStopCarousel(){
        return new StopCarousel(carousel, telemetry);
    }

    public MoveCarousel getMoveCarouselRight(){
        return moveCarouselRight;
    }

    public StopCarousel getStopCarousel(){
        return stopCarousel;
    }

    public boolean hasMetMaxEncoderCount(){
        return carousel.getCarouselCurrentPosition() >= MAX_ENCODER_COUNT;
    }
}
