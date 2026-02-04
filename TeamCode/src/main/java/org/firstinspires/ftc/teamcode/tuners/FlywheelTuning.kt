package org.firstinspires.ftc.teamcode.tuners

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor

@Configurable
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
class FlywheelTuning : LinearOpMode() {
    private val panelsTelemetry = PanelsTelemetry.telemetry
    private var batteryVoltageSensor: VoltageSensor? = null

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val flywheel = hardwareMap.get(DcMotorEx::class.java, FLYWHEEL_MOTOR_NAME)
        flywheel.direction = if (FLYWHEEL_REVERSED) {
            DcMotorSimple.Direction.REVERSE
        } else {
            DcMotorSimple.Direction.FORWARD
        }
        flywheel.mode = DcMotor.RunMode.RUN_USING_ENCODER

        val tuningController = TuningController()

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        setPIDFCoefficients(flywheel, MOTOR_VELO_PID)

        var lastKp = 0.0
        var lastKi = 0.0
        var lastKd = 0.0
        var lastKf = motorVelocityF

        panelsTelemetry.addLine("Ready!")
        panelsTelemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val targetVelo = tuningController.update()
            flywheel.velocity = targetVelo

            panelsTelemetry.addData("targetVelocity", targetVelo)
            panelsTelemetry.addData("velocity", flywheel.velocity)
            panelsTelemetry.addData("error", targetVelo - flywheel.velocity)
            panelsTelemetry.addData("motorVelocityF", motorVelocityF)
            panelsTelemetry.addData("p", MOTOR_VELO_PID.p)
            panelsTelemetry.addData("i", MOTOR_VELO_PID.i)
            panelsTelemetry.addData("d", MOTOR_VELO_PID.d)
            panelsTelemetry.addData("f", MOTOR_VELO_PID.f)

            panelsTelemetry.addData(
                "upperBound",
                TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15)
            )
            panelsTelemetry.addData("lowerBound", 0)

            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(flywheel, MOTOR_VELO_PID)

                lastKp = MOTOR_VELO_PID.p
                lastKi = MOTOR_VELO_PID.i
                lastKd = MOTOR_VELO_PID.d
                lastKf = MOTOR_VELO_PID.f
            }

            tuningController.update()
            panelsTelemetry.update()
        }
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

    companion object {
        var FLYWHEEL_MOTOR_NAME: String = "rfw"
        var FLYWHEEL_REVERSED: Boolean = true

        var MOTOR_VELO_PID: PIDFCoefficients = PIDFCoefficients(150.0, 0.0, 5.0, 17.0)

        val motorVelocityF: Double
            get() = 32767 * 60.0 / (TuningController.MOTOR_MAX_RPM * TuningController.MOTOR_TICKS_PER_REV)
    }
}
