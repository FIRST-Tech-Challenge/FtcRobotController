//package org.firstinspires.ftc.teamcodekt.xteasting
//
//import org.reflections.Reflections
//import java.lang.reflect.Field
//import java.lang.reflect.Modifier
//
//annotation class Teast
//
//@Teast
//object MoerTeasting {
//    init {
//        println("HI")
//    }
//}
//
//fun main() {
//    // Fuck you, slf4j
//    val f = Reflections::class.java.getField("log")
//    f.isAccessible = true
//
//    val modifiersField: Field = Field::class.java.getDeclaredField("modifiers")
//    modifiersField.isAccessible = true
//    modifiersField.setInt(f, f.modifiers and Modifier.FINAL.inv())
//
//    f.set(null, null)
//
//    val ts = Reflections("org.firstinspires.ftc.teamcodekt.xteasting")
//        .getTypesAnnotatedWith(Teast::class.java)
//}
