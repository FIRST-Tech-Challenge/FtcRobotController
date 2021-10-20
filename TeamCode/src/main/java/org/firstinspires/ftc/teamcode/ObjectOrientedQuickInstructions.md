<!-- If you are seeing this it probably isn't the best way to read this document you can check it out @ https://github.com/FTC14133/FTC14133-2021-2022/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/JakeInstructions.md -->

# General Structure

To split up your coding into more than one file you must implement object oriented programming concepts

## Your files

you can make as many files as for this example we will look at drive train but you can take this idea and apply it around to other aspects of the robot as well.

This file should only handle the logic as it pertains to the drive train and nothing else

It should include an `Update` function that passes whatever values that you need this can be a controller, or calculated values or whatever you need.

```java
// DriveTrain.java

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain{
    private DcMotorEx lb = null;        // Sets the variables of the mecanum wheels
    private DcMotorEx rb = null;
    private DcMotorEx lf = null;
    private DcMotorEx rf = null;
    
    public Drivetrain(){
        lb = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "lb");
        rb = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "rb");
        lf = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "lf");       //sets the names of the motors on the hardware map
        rf = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "rf");

        lf.setDirection(DcMotorEx.Direction.FORWARD);
        rf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.REVERSE);
    }


    // I have no idea what class gamepad and I can't analyze it on my current rig so we are gonna just pretend that its called GamePad 
    public void Update(GamePad gp){
        
        var leftPowerY = -gp.left_stick_y;      //find the value of y axis on the left joystick
        var leftPowerX = gp.left_stick_x;      //find the value of x axis on the left joystick
        var rightPowerX = gp.right_stick_x;      //find the value of x axis on the right joystick

        var leftFrontPower = leftPowerY + leftPowerX - rightPowerX;     //Power of Mecanum wheels
        var rightFrontPower = leftPowerY - leftPowerX + rightPowerX;
        var leftBackPower = leftPowerY - leftPowerX - rightPowerX;
        var rightBackPower = leftPowerY + leftPowerX + rightPowerX;

        this.lf.setPower(leftFrontPower);     //Sets the power of the wheels
        this.lb.setPower(leftBackPower);
        this.rf.setPower(rightFrontPower);
        this.rb.setPower(rightBackPower);
    }
}
```

Now we have the code that executes inside your `FTC_xxxxx_yyyy.java` file instead of making motors and assigning the values in there we will let the new `DriveTrain` class handle it instead. So the new file will look something like this

```java
@TeleOp(name="FTC xxxxx yyyy", group="Iterative Opmode")
@Disabled
public class FTC_xxxxx_yyyy extends OpMode {
    private DriveTrain driveTrain = null;
    
    public void init(){
        driveTrain = new DriveTrain();
    }

    public void init_loop() {}

    public void start() {}

    public void loop(){
        driveTrain.Update(gamepad1);
    }
}
```

this was a brief of example of how these things can work, one thing that is common is to use a class to handle all of your configurations which can be passed into the constructors of the class as need be.