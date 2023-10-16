# Gamepadyn: An FTC Input Action library

## About

## Getting Started

Samples are available in the "samples" directory. Place them in your TeamCode folder alongside your OpModes.

TODO: add real documentation

## Generic Terminology

- FTC = First Tech Challenge
- FTC &lt;something&gt; class = A Java class from the FTC SDK
- Gamepad = A "video game controller" (you know what that is)
- Input device = A source of raw input, always (for now) a gamepad
- Analog input = An input represented as a number or a set of numbers 
- Digital input = An input represented as a boolean

## Architecture

### Gamepadyn (manager)

NOTE: maybe consider renaming the class to "Manager"? it makes more sense from a high-level standpoint.

To start using Gamepadyn, create an instance of the Gamepadyn class.
You should probably store the instance as a member of the OpMode/superclass using it,
but we're not stopping you from doing something different.

### Actions

An Action is something your robot can do.
This is broad and ambiguous on purpose, as to afford more flexibility for FTC developers.
Here's an example of what your actions could look like:

```kotlin
enum class TestAction {
    MOVEMENT,   // Analog 2D (X/Y)
    ROTATION,   // Analog 1D (yaw)
    OPEN_CLAW,  // Digital
    CLOSE_CLAW; // Digital
}
```

### Configurations and Bindings

A Configuration is a set of Bindings.
A Binding is a map between a raw gamepad input to an action/actions.
A simple Binding is a good Binding- don't overcomplicate things.
Try to operate under the philosophy of "one button per action,"
as stacking multiple inputs for single actions may confuse the input handler.

### Player

The Player class is an abstraction of an FTC Gamepad class.
It can reasonably be assumed that two players won't be using the same controller,
so we operate under that.

Each player instance holds a Configuration. Configurations can be used by multiple players, as they're really just data.

## Type Details

### RawInput

An enumeration of the different inputs of an input device

### ActionType

An enumeration of an input's type (`ANALOG` or `DIGITAL`)

### ActionData\[Analog/Digital\]

A representation of an Analog or Digital action.
