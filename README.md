## Control Scheme
The control scheme can be found here: [Control Schemes](https://www.google.com/url?q=https://www.padcrafter.com/?templates%3DGamepad%2B2%257CGamepad%2B1%26aButton%3DToggle%2BClaw%26rightBumper%3D%26col%3D%2523242424%252C%2523606A6E%252C%2523FFFFFF%26rightTrigger%3DExtend%2BDrawer%26leftTrigger%3DRetract%2BDrawer%26bButton%3D%26xButton%3DToggle%2BClaw%2BJoint%26plat%3D%257C%257C0%26rightStick%3D%257CTurn%26leftStickClick%3D%257CMove&sa=D&source=docs&ust=1730486755886136&usg=AOvVaw08bQ9yp8JotVR7uCbEcO9a) (you might have to zoom out to view the binds on the controller)

## Structure
The robot code follows an [composition over inheritance](https://en.wikipedia.org/wiki/Composition_over_inheritance) structure (emphasizing has-a over is-a relationships). 

The `RobotTeleopMecanumDrive.java` is the primary class that holds the objects related to teleop. Currently, it contains a `Mecanum`, `Drawer`, `Elevator`, and `SpecimanArm`.

## Best Practices
- Try to follow a composition over inheritance structure when implementing new classes.
- Follow good naming practices (perferably via a style guide)
- For anything that needs to move based on input, create a method `.update(Gamepad gamepad)` and put your movement code there.
- It's a good idea to have either a `toString()` or some other method of telemetry to help debug your classes.
- Reading up on FTC documentation and doing your own research is the best way to learn.

## Miscellaneous
## Vector2
### Definition
The `Vector2` class represents a vector in 2D space with a magnitude a direction. It has a variety of methods such as `Vector2.isUnitVector()`, `Vector2.getMagnitude()`, and `Vector2.normalize()`.
### Rationale
The point of this class is to generalize inputs into a vector. By doing this, it is easy to use for both teleop and autonomous.
