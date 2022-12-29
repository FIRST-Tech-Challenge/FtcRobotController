# LateInitVal

A delegate class used to simulate a lateinit value; any variable that delegates to this may
only be initialized once, but it may be initialized whenever it is desired.

It's a psuedo-`lateinit val` basically, which Kotlin doesn't allow.

*NOTE: This only works for class members.*

Also, **this is not thread safe.** Thread safety will be implemented if required.

```kotlin
class IntegerThatCanOnlyBeSetOnce {
  var int: Int by LateInitVal()
  // or
  var int by LateInitVal<Int>()
}

//...

fun test1() {
  val myInt = IntegerThatCanOnlyBeSetOnce()
  myInt.int = 1 // this is fine
  myInt.int = 2 // throws an exception; int has already been initialized
}

fun test2() {
  val myInt = IntegerThatCanOnlyBeSetOnce()
  println(myInt.int) // throws an exception; int is uninitialized
}
```

*Motivation:* When reading through Rogue's opmodes from last year, a major issue was that
I simply could not tell which variables were genuinely intended to be modified and which
ones were effectively final. This helps makes intentions more clear from the get-go.
