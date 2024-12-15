package dev.aether.collaborative_multitasking_tests
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.SharedResource
import dev.aether.collaborative_multitasking.TaskGroup
import org.junit.jupiter.api.Assertions.assertEquals
import kotlin.test.BeforeTest
import kotlin.test.Test

internal class TestTaskGroups {
    companion object {
        private val LOCK1 = SharedResource("lock1")
        private val LOCK2 = SharedResource("lock2")
    }

    private lateinit var scheduler: MultitaskScheduler

    @BeforeTest
    fun prepare() {
        scheduler = MultitaskScheduler()
    }

    @Test
    fun `Test empty task group init`() {
        scheduler.add(TaskGroup(scheduler).with {})
    }

    @Test
    fun `Test task group runs tasks`() {
        var count = 0
        scheduler.add(TaskGroup(scheduler).with {
            it.task {
                onStart { -> count++ }
                isCompleted { -> true }
            }.then {
                onStart { -> count++ }
                isCompleted { -> true }
            }.then {
                onStart { -> count++ }
                isCompleted { -> true }
            }
        })
        scheduler.runToCompletion { true }
        assertEquals(count, 3) { "Did not run all 3 tasks" }
    }

    @Test
    fun `Test task group owns union of locks`() {
        val theGroup = TaskGroup(scheduler).with {
            it.task {
                +LOCK1
                isCompleted { -> false }
            }
            it.task {
                canStart { -> false }
                +LOCK2
                isCompleted { -> false }
            }
        }
        scheduler.add(theGroup)
        println("[group]: ${theGroup.name}: ${theGroup.requirements()}")
        assert(theGroup.requirements().let {
            it.contains(LOCK1) && it.contains(LOCK2)
        }) { "Locks not unioned - actually ${theGroup.requirements()} for ${theGroup.name}" }
        scheduler.tick()
        assert(scheduler.isResourceInUse(LOCK1)) { "Lock1 not in use" }
        assert(scheduler.isResourceInUse(LOCK2)) { "Lock2 not in use" }
        val inner = theGroup.accessInner()
        assert(inner.isResourceInUse(LOCK1)) { "Lock1 not in use inside" }
        assert(!inner.isResourceInUse(LOCK2)) { "Lock2 in use inside (should be unused)" }
    }
}