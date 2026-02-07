package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import kotlin.math.abs

class FlywheelShooter(
    val rightMotorName: String,
    val leftMotorName: String,
    val rightDirection: DcMotorSimple.Direction,
    val leftDirection: DcMotorSimple.Direction,
) {
    companion object {
        @JvmField var MOTOR_VELO_PID_RIGHT: PIDFCoefficients = PIDFCoefficients(150.0, 0.0, 0.0, 17.0)
        @JvmField var MOTOR_VELO_PID_LEFT: PIDFCoefficients = PIDFCoefficients(150.0, 0.0, 0.0, 18.0)
    }

    private var batteryVoltageSensor: VoltageSensor? = null
    private lateinit var rightFlywheel: DcMotorEx
    private lateinit var leftFlywheel: DcMotorEx

    fun init(hardwareMap: HardwareMap) {
        rightFlywheel = hardwareMap.get(DcMotorEx::class.java, rightMotorName).apply {
            direction = rightDirection
            mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        leftFlywheel = hardwareMap.get(DcMotorEx::class.java, leftMotorName).apply {
            direction = leftDirection
            mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        setPIDFCoefficients(rightFlywheel, MOTOR_VELO_PID_RIGHT)
        setPIDFCoefficients(leftFlywheel, MOTOR_VELO_PID_LEFT)
    }

    fun setVelocity(velocity: Double) {
        // Velocity is in motor ticks per second (DcMotorEx velocity units).
        rightFlywheel.velocity = velocity
        leftFlywheel.velocity = velocity
    }

    fun getVelocity(): Double {
        return (abs(rightFlywheel.velocity) + abs(leftFlywheel.velocity)) / 2.0
    }

    fun stop() {
        rightFlywheel.power = 0.0
        leftFlywheel.power = 0.0
    }

    private fun setPIDFCoefficients(motor: DcMotorEx, coefficients: PIDFCoefficients) {
        val batteryVoltage = batteryVoltageSensor?.voltage ?: 12.0
        motor.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            PIDFCoefficients(
                coefficients.p,
                coefficients.i,
                coefficients.d,
                coefficients.f * 12 / batteryVoltage
            )
        )
    }
}
