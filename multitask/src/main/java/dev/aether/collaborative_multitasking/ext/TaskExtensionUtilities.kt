@file:Suppress("unused")

package dev.aether.collaborative_multitasking.ext

import dev.aether.collaborative_multitasking.Runnable
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.TaskAction1
import dev.aether.collaborative_multitasking.TaskAction2
import dev.aether.collaborative_multitasking.TaskQuery1
import dev.aether.collaborative_multitasking.TaskQuery2

fun Task.extendOnStart(other: Runnable) {
    val original = this.onStart
    onStart { a, b ->
        original(a, b)
        other()
    }
}
fun Task.extendOnStart(other: TaskAction1) {
    val original = this.onStart
    onStart { a, b ->
        original(a, b)
        other(a)
    }
}
fun Task.extendOnStart(other: TaskAction2) {
    val original = this.onStart
    onStart { a, b ->
        original(a, b)
        other(a, b)
    }
}

fun Task.isCompletedAnd(other: () -> Boolean) {
    val original = this.isCompleted
    isCompleted { a, b ->
        original(a, b) && other()
    }
}
fun Task.isCompletedAnd(other: TaskQuery1<Boolean>) {
    val original = this.isCompleted
    isCompleted { a, b ->
        original(a, b) && other(a)
    }
}
fun Task.isCompletedAnd(other: TaskQuery2<Boolean>) {
    val original = this.isCompleted
    isCompleted { a, b ->
        original(a, b) && other(a, b)
    }
}

fun Task.isCompletedOr(other: () -> Boolean) {
    val original = this.isCompleted
    isCompleted { a, b ->
        original(a, b) || other()
    }
}
fun Task.isCompletedOr(other: TaskQuery1<Boolean>) {
    val original = this.isCompleted
    isCompleted { a, b ->
        original(a, b) || other(a)
    }
}
fun Task.isCompletedOr(other: TaskQuery2<Boolean>) {
    val original = this.isCompleted
    isCompleted { a, b ->
        original(a, b) || other(a, b)
    }
}

fun Task.canStartAnd(other: () -> Boolean) {
    val original = this.canStart
    canStart { a, b ->
        original(a, b) || other()
    }
}
fun Task.canStartAnd(other: TaskQuery1<Boolean>) {
    val original = this.canStart
    canStart { a, b ->
        original(a, b) || other(a)
    }
}
fun Task.canStartAnd(other: TaskQuery2<Boolean>) {
    val original = this.canStart
    canStart { a, b ->
        original(a, b) || other(a, b)
    }
}

fun Task.canStartOr(other: () -> Boolean) {
    val original = this.canStart
    canStart { a, b ->
        original(a, b) || other()
    }
}
fun Task.canStartOr(other: TaskQuery1<Boolean>) {
    val original = this.canStart
    canStart { a, b ->
        original(a, b) || other(a)
    }
}
fun Task.canStartOr(other: TaskQuery2<Boolean>) {
    val original = this.canStart
    canStart { a, b ->
        original(a, b) || other(a, b)
    }
}
