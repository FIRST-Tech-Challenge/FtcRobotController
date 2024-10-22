package auto;

public class Robot {
    private Chassis chassis;
    private HSlide hSlide;
    private VSlide vSlide;

    public Robot() {
        chassis = new Chassis();
        hSlide = new HSlide();
        vSlide = new VSlide();
    }

    public void resetAll() {
        chassis.reset();
        hSlide.resetPosition();
        vSlide.resetPosition();
    }
}