package org.firstinspires.ftc.teamcode.rework.ModuleTools;


public interface Module {
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/rework/ModuleTools/Module.java
    //TODO implement boolean flag for turning module on and off

=======
>>>>>>> 661b8a8450127843346bf11f914073b604a851b6:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/rework/Robot/Modules/Module.java
    /**
     * Initializes the module. This includes setting up all motors/servos
     * */
    public void init();

    /**
     * Updates the module, executing all the tasks it should complete on every iteration,
     * utilizing the module's states. This separate method allows for each module to be updated
     * on a different thread from where the states are set.
     */
    public void update();
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/rework/ModuleTools/Module.java

    public void tellEm();
=======
>>>>>>> 661b8a8450127843346bf11f914073b604a851b6:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/rework/Robot/Modules/Module.java
}
