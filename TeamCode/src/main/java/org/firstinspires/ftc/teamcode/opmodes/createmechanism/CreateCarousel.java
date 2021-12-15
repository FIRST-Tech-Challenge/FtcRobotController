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
import org.firstinspires.ftc.teamcode.subsystems.leds.blinkin.LEDSubsystem;

public class CreateCarousel {

    private CarouselSubsystem carousel;
    private final HardwareMap hwMap;
    private final String deviceName;
    private final Telemetry telemetry;
    private final GamepadEx op;

    private static final double MOVE_RIGHT_POWER = 0.5;
    private static final double MOVE_LEFT_POWER = -0.5;

    public CreateCarousel (final HardwareMap hwMap, final String deviceName, final GamepadEx op, Telemetry telemetry){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.op = op;
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

        carousel = new CarouselSubsystem(hwMap,"carousel");

        MoveCarousel moveCarouselRight = new MoveCarousel(carousel,MOVE_RIGHT_POWER, telemetry);
        MoveCarousel moveCarouselLeft = new MoveCarousel(carousel, MOVE_LEFT_POWER, telemetry);
        StopCarousel stopCarousel = new StopCarousel(carousel, telemetry);

        Button carouselRight = new GamepadButton(op, GamepadKeys.Button.RIGHT_BUMPER);
        Button carouselLeft = new GamepadButton(op, GamepadKeys.Button.LEFT_BUMPER);


        carouselRight.whileHeld(moveCarouselRight);
        carouselLeft.whileHeld(moveCarouselLeft);
        carousel.setDefaultCommand(new PerpetualCommand(stopCarousel));

    }
}
