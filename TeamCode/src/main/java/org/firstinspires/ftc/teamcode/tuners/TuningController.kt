package org.firstinspires.ftc.teamcode.tuners

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.util.ElapsedTime

@Configurable
class TuningController {
    internal enum class State {
        RAMPING_UP,
        COASTING_1,
        RAMPING_DOWN,
        COASTING_2,
        RANDOM_1,
        RANDOM_2,
        RANDOM_3,
        REST
    }

    private var currentState: State = State.RAMPING_UP
    private val externalTimer = ElapsedTime()
    private var onEnter = true
    private var currentTargetVelo = 0.0

    fun update(): Double {
        when (currentState) {
            State.RAMPING_UP -> {
                if (onEnter) {
                    externalTimer.reset()
                    onEnter = false
                }
                if (externalTimer.seconds() >= ZSTATE1_RAMPING_UP_DURATION) {
                    onEnter = true
                    currentState = State.COASTING_1
                }
                val progress = externalTimer.seconds() / ZSTATE1_RAMPING_UP_DURATION
                val target = progress * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED

                currentTargetVelo = rpmToTicksPerSecond(target)
            }

            State.COASTING_1 -> {
                if (onEnter) {
                    externalTimer.reset()
                    onEnter = false
                    currentTargetVelo = rpmToTicksPerSecond(TESTING_MAX_SPEED)
                }
                if (externalTimer.seconds() >= ZSTATE2_COASTING_1_DURATION) {
                    onEnter = true
                    currentState = State.RAMPING_DOWN
                }
            }

            State.RAMPING_DOWN -> {
                if (onEnter) {
                    externalTimer.reset()
                    onEnter = false
                }
                if (externalTimer.seconds() >= ZSTATE3_RAMPING_DOWN_DURATION) {
                    onEnter = true
                    currentState = State.COASTING_2
                }
                val progress = externalTimer.seconds() / ZSTATE3_RAMPING_DOWN_DURATION
                val target = TESTING_MAX_SPEED - progress * (TESTING_MAX_SPEED - TESTING_MIN_SPEED)

                currentTargetVelo = rpmToTicksPerSecond(target)
            }

            State.COASTING_2 -> {
                if (onEnter) {
                    externalTimer.reset()
                    onEnter = false
                    currentTargetVelo = rpmToTicksPerSecond(TESTING_MIN_SPEED)
                }
                if (externalTimer.seconds() >= ZSTATE4_COASTING_2_DURATION) {
                    onEnter = true
                    currentState = State.RANDOM_1
                }
            }

            State.RANDOM_1 -> {
                if (onEnter) {
                    externalTimer.reset()
                    onEnter = false
                    currentTargetVelo = rpmToTicksPerSecond(
                        Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED
                    )
                }
                if (externalTimer.seconds() >= ZSTATE5_RANDOM_1_DURATION) {
                    onEnter = true
                    currentState = State.RANDOM_2
                }
            }

            State.RANDOM_2 -> {
                if (onEnter) {
                    externalTimer.reset()
                    onEnter = false
                    currentTargetVelo = rpmToTicksPerSecond(
                        Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED
                    )
                }
                if (externalTimer.seconds() >= ZSTATE6_RANDOM_2_DURATION) {
                    onEnter = true
                    currentState = State.RANDOM_3
                }
            }

            State.RANDOM_3 -> {
                if (onEnter) {
                    externalTimer.reset()
                    onEnter = false
                    currentTargetVelo = rpmToTicksPerSecond(
                        Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED
                    )
                }
                if (externalTimer.seconds() >= ZSTATE7_RANDOM_3_DURATION) {
                    onEnter = true
                    currentState = State.REST
                }
            }

            State.REST -> {
                if (onEnter) {
                    externalTimer.reset()
                    onEnter = false
                    currentTargetVelo = 0.0
                }
                if (externalTimer.seconds() >= ZSTATE8_REST_DURATION) {
                    onEnter = true
                    currentState = State.RAMPING_UP
                }
            }
        }

        return currentTargetVelo
    }

    companion object {
        @JvmField var MOTOR_TICKS_PER_REV: Double = 28.0
        @JvmField var MOTOR_MAX_RPM: Double = 4500.0
        @JvmField var MOTOR_GEAR_RATIO: Double = 0.75 // output (wheel) speed / input (motor) speed

        @JvmField var TESTING_MAX_SPEED: Double = 0.5 * MOTOR_MAX_RPM
        @JvmField var TESTING_MIN_SPEED: Double = 0.1 * MOTOR_MAX_RPM

        // These are prefixed with "STATE1", "STATE2", etc. because Dashboard displays variables in
        // alphabetical order. Thus, we preserve the actual order of the process
        // Then we append Z just because we want it to show below the MOTOR_ and TESTING_ because
        // these settings aren't as important
        @JvmField var ZSTATE1_RAMPING_UP_DURATION: Double = 3.5
        @JvmField var ZSTATE2_COASTING_1_DURATION: Double = 4.0
        @JvmField var ZSTATE3_RAMPING_DOWN_DURATION: Double = 2.0
        @JvmField var ZSTATE4_COASTING_2_DURATION: Double = 2.0
        @JvmField var ZSTATE5_RANDOM_1_DURATION: Double = 2.0
        @JvmField var ZSTATE6_RANDOM_2_DURATION: Double = 2.0
        @JvmField var ZSTATE7_RANDOM_3_DURATION: Double = 2.0
        @JvmField var ZSTATE8_REST_DURATION: Double = 1.0

        fun rpmToTicksPerSecond(rpm: Double): Double {
            return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60
        }
    }
}
