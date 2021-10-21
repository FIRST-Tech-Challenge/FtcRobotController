<!-- If you are seeing this it probably isn't the best way to read this document you can check it out @ https://github.com/FTC14133/FTC14133-2021-2022/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ObjectOrientedQuickInstructions.md -->

# General Structure

To split up your coding into more than one file you must implement object oriented programming concepts

## Your files

You can make as many files as for this example we will look at drive train but you can take this idea and apply it around to other aspects of the robot as well.

This file should only handle the logic as it pertains to the drive train and nothing else

It should include an `Update` function that passes whatever values that you need this can be a controller, or calculated values or whatever you need.

You can copy this code, r-click, and refactor>rename to your subsystem and each hardware you wish to have

```java
// Subsystem.java

import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Subsystem {
    private DcMotorEx subsystemmotor1 = null;        // Sets the variables of the motors and other hardware
    private DcMotorEx subsystemmotor2 = null;
    private Servo subsystemservo1 = null;
    private DigitalChannel digitalsensor1 = null;
    private AnalogSensor analogsensor1 = null;

    // read in the hardware map from the main
    public Subsystem(HardwareMap hardwareMap) {
        subsystemmotor1 = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "subsystemmotor1"); //sets the names of the motors on the hardware map
        subsystemmotor2 = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "subsystemmotor2");
        subsystemservo1 = (Servo) hardwareMap.get(Servo.class, "subsystemservo1");  //Use this for servos     
        digitalsensor1 = (DigitalChannel) hardwareMap.get(DigitalChannel.class, "digitalsensor1"); //Use this for digital sensors

        subsystemmotor1.setDirection(DcMotorEx.Direction.FORWARD);
        subsystemmotor2.setDirection(DcMotorEx.Direction.REVERSE);
    }
    
    public void Update(gamepad gp) {
    // Run the looping code here using the instated hardware above    
    }
}
```

Now we have the code that executes inside your `FTC_xxxxx_yyyy.java` file instead of making motors and assigning the values in there we will let the new `Subsystem` class handle it instead. So the new file will look something like this

```java
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

@TeleOp(name = "FTC_xxxxx_yyyy", group = "Iterative Opmode")
@Disabled
public class FTC_xxxxx_yyyy extends OpMode {
    private Subsystem subsystem = null;

    public void init() {
        // pass the hardware map into the constructor
        subsystem = new Subsystem(hardwareMap); //Pushes the map from the subsystem to the bot
    }

    public void init_loop() {
    }

    public void start() {
    }

    public void loop() {
        subsystem.Update(gamepad1); // Runs the subsystem code
    }
}
```

This was a brief of example of how these things can work, one thing that is common is to use a class to handle all of your configurations which can be passed into the constructors of the class as need be.

## Further separating data

Java is an old programming language and sometimes we must accommodate for this age. To separate Java files into different scripts you must use packages

### A brief word on packages

A package is simply a folder with additional meta-properties that allow the JVM (Java Virtual Machine) to see and load code from. If you want to learn more just talk to Jake

### Creating packages

Most Development Environments will allow you to create new packages by navigating to the `File` option bar from there its `File -> new -> package` and then you give it a package name

### Using your package

your package will consist of java files for this example we will have a package called `myCode` which will also include the file `DriveTrain.java` so the file paths look something like this

```bash
teamCode/
    FTC_xxxxx_yyyy.java
    myCode/
        DriveTrain.java
```

And we can use the code back in `FTC_xxxxx_yyyy.java` by updating the import to look like this

```java
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
```

The rest of the code can remain the same.