package org.firstinspires.ftc.teamcode.robot

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.ShooterConfig.kD
import org.firstinspires.ftc.teamcode.robot.ShooterConfig.kF
import org.firstinspires.ftc.teamcode.robot.ShooterConfig.kI
import org.firstinspires.ftc.teamcode.robot.ShooterConfig.kP
import org.firstinspires.ftc.teamcode.robot.ShooterConfig.targetRpm
import kotlin.math.max
import kotlin.math.min

@Configurable
object ShooterConfig{

    @JvmField var kP: Double = 0.0025
    @JvmField var kI: Double = 0.0
    @JvmField var kD: Double = 0.0
    @JvmField var kF: Double = 0.0

    @JvmField var targetRpm: Double = 0.0
}
class ShooterPID(kP1: Double, kI1: Double, kD1: Double, kF1: Double) {

    private lateinit var leftFlywheel: DcMotorEx
    private lateinit var rightFlywheel: DcMotorEx

    private var leftLastError = 0.0
    private var rightLastError = 0.0
    private var leftIntegral = 0.0
    private var rightIntegral = 0.0


    fun init(hardwareMap: HardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotorEx::class.java, "leftFlywheel").apply {
            mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        rightFlywheel = hardwareMap.get(DcMotorEx::class.java, "rightFlywheel").apply {
            mode = DcMotor.RunMode.RUN_USING_ENCODER
            direction = DcMotorSimple.Direction.REVERSE
        }

    }

    fun setTargetRpm(rpm: Double) {
        targetRpm = rpm
    }

    fun update() {
        val targetTicksPerSecond = (targetRpm / 60.0) * TICKS_PER_REV

        // --- Left motor ---
        val leftVelocity = leftFlywheel.velocity
        val leftError = targetTicksPerSecond - leftVelocity
        leftIntegral += leftError
        val leftDerivative = leftError - leftLastError
        leftLastError = leftError

        val leftOutput = (kP * leftError) +
                (kI * leftIntegral) +
                (kD * leftDerivative) +
                (kF * targetTicksPerSecond)
        val leftClipped = min(max(leftOutput, -.75), .75)

        // --- Right motor ---
        val rightVelocity = rightFlywheel.velocity
        val rightError = targetTicksPerSecond - rightVelocity
        rightIntegral += rightError
        val rightDerivative = rightError - rightLastError
        rightLastError = rightError

        val rightOutput = (kP * rightError) +
                (kI * rightIntegral) +
                (kD * rightDerivative) +
                (kF * targetTicksPerSecond)
        val rightClipped = min(max(rightOutput, -.75), .75)

        // Apply power
        leftFlywheel.power = leftClipped
        rightFlywheel.power = rightClipped
    }

    fun flywheel() {
        rightFlywheel.targetPosition
        leftFlywheel.targetPosition

    }

    fun stop() {
        leftFlywheel.power = 0.0
        rightFlywheel.power = 0.0
        leftIntegral = 0.0
        rightIntegral = 0.0
        leftLastError = 0.0
        rightLastError = 0.0
        targetRpm = 0.0
    }


    // goBILDA 6000rpm motor encoder resolution
    val TICKS_PER_REV = 28.0
}