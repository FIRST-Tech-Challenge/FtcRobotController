# Problem Statement

When `Intake` lowers, `Arm` needs to move into shooting postion

- `Intake.lower` is a method of the  `Intake` class which lowers the intake.
- `Arm.ready` is a method of the `Arm` class that ready's the arm.
- `Controller.button5` is a member of the Controller denoting when to execute the above code.

## Method 1

Link both methods to the same input statement.

So in your controller executing class you would have a code snippet similar to this

```java
// or isPressed() or however you denote a button being pressed
if(Controller.button5 == 1){
    Intake.lower();
    Arm.ready();
}
```

This is the easiest method but sometimes it can not be optimal for instances when aspects of these instructions need to run in tandem that may lead to an issue so there are other options as well

## Method 2 One-Way Binding

This method is to be used when `Intake` needs some control over `Arm` but `Arm` doesn't require any control over `Intake`. When this is the scenario, the __One-Way Binding__ is the way to go.

Assign `Arm` as a member of `Intake` in the constructor, and then use as needed in the update

```java
//Intake.java
public Intake(Arm arm){
    //NOTE we do not use the constructor we give the actual object
    this.arm = arm 
}

public void update(){
    //here we do a smidgen of Intake stuff
    this.arm.ready(); 
    //here we do another smidgen of Intake stuff
}
```

Now if `Arm` needs to control `Intake` than we run into an issue of circular dependicies, where the code will continually run each other over and over again and robot will fail to move. Thus leads us to method 3 __Nth-way Bindings__

## Nth-way (2 or more - way) Bindings

In this use case we can't assign either as members of the other so instead we use [__pass by reference__](https://medium.com/swlh/java-passing-by-value-or-passing-by-reference-c75e312069ed) to control the objects that are being passed back and forth.

```java
//Intake.java
public void update(Controls controls, Arm arm){
    if(controls.button5 == 1){
        intake.goToPosition1();
        arm.ready();
        intake.goToPosition2();
    }
}
```

```java
//Arm.java
public void update(Controls controls, Intake intake){
    if(controls.buttonTrigger == 1){
        this.fire();
        intake.reset();
        this.resetArm();
    }
}
```