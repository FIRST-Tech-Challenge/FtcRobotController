# BetterHwMapGet

Syntactic sugar for the typical [hardwareMap.get()](https://gm0.org/en/latest/docs/software/getting-started/common-hardware-components.html#examples-of-using-common-hardware-components)

Basically, due to type erasure with Java generics, when using `HardwareMap.get(Class::class, "name")`, you need to
manually pass in the class of the component you want to get, e.g.

```kotlin
val motor = hardwareMap.get(DcMotorEx::class.java, "motor")
```

Using reified types and operator overloading, we can simply do this instead:

```kotlin
val motor = hardwareMap<DcMotorEx>("motor")
//or
val motor: DcMotorEx = hardwareMap("motor")
```

*Also, for why I did not simply use `get`, it's because the `get` name is already taken by the `HardwareMap` class :(*

And yes, this just makes me feel cool. Shut up.

:::tip
If you try to type in `hardwareMap<*>("whatever")` and the IDE is giving you a hard time trying to import the right
method, just type in `hardwareMap.invoke` until it auto-completes for you and imports the right method.

Then you can just delete `.invoke` and you're good to go.

This is because this trick is done by overriding the `invoke` operator, which is a special operator in Kotlin.
:::
