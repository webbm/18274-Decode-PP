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
object IntakeConfig{

    @JvmField var kP: Double = 0.0025
    @JvmField var kI: Double = 0.0
    @JvmField var kD: Double = 0.0
    @JvmField var kF: Double = 0.0

    @JvmField var targetRpm: Double = 0.0
}
class IntakePID(kP1: Double, kI1: Double, kD1: Double, kF1: Double) {

    private lateinit var leftIntake: DcMotorEx
    private lateinit var rightIntake: DcMotorEx

    private var leftLastError = 0.0
    private var rightLastError = 0.0
    private var leftIntegral = 0.0
    private var rightIntegral = 0.0


    fun init(hardwareMap: HardwareMap) {
        leftIntake = hardwareMap.get(DcMotorEx::class.java, "left_intake").apply {
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            direction = DcMotorSimple.Direction.FORWARD
        }
        rightIntake = hardwareMap.get(DcMotorEx::class.java, "right_intake").apply {
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            direction = DcMotorSimple.Direction.FORWARD
        }

    }

    fun setTargetRpm(rpm: Double) {
        targetRpm = rpm
    }

    fun update() {
        val targetTicksPerSecond = (targetRpm / 60.0) * TICKS_PER_REV

        // --- Left motor ---
        val leftVelocity = leftIntake.velocity
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
        val rightVelocity = rightIntake.velocity
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
        leftIntake.power = leftClipped
        rightIntake.power = rightClipped
    }

    fun flywheel() {
        rightIntake.targetPosition
        leftIntake.targetPosition

    }

    fun getFlywheelRpm(): Double {
        val leftRpm = (leftIntake.velocity / TICKS_PER_REV) * 60.0
        val rightRpm = (rightIntake.velocity / TICKS_PER_REV) * 60.0
        return (leftRpm + rightRpm) / 2.0
    }

    fun stop() {
        leftIntake.power = 0.0
        rightIntake.power = 0.0
        leftIntegral = 0.0
        rightIntegral = 0.0
        leftLastError = 0.0
        rightLastError = 0.0
        targetRpm = 0.0
    }


    // goBILDA 1150rpm motor encoder resolution
    val TICKS_PER_REV = 145.1
}
