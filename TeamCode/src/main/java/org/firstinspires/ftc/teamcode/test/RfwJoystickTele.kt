package org.firstinspires.ftc.teamcode.test

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.Range
import kotlin.math.abs

@Configurable
@TeleOp(name = "RFW Joystick Tele", group = "Test")
class RfwJoystickTele : LinearOpMode() {

    companion object {
        var MOTOR_VELO_PID_RIGHT: PIDFCoefficients = PIDFCoefficients(150.0, 0.0, 0.0, 17.0)
        var MOTOR_VELO_PID_LEFT: PIDFCoefficients = PIDFCoefficients(150.0, 0.0, 0.0, 18.0)
    }

    private var batteryVoltageSensor: VoltageSensor? = null
    private val panelsTelemetry = PanelsTelemetry.telemetry
    private val velocityTolerance = 75.0
    override fun runOpMode() {
        val rfw = hardwareMap.get(DcMotorEx::class.java, "rfw")
        val lfw = hardwareMap.get(DcMotorEx::class.java, "lfw")
        rfw.direction = DcMotorSimple.Direction.REVERSE
        lfw.direction = DcMotorSimple.Direction.FORWARD
        rfw.mode = DcMotor.RunMode.RUN_USING_ENCODER
        lfw.mode = DcMotor.RunMode.RUN_USING_ENCODER
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        setPIDFCoefficients(rfw, MOTOR_VELO_PID_RIGHT)
        setPIDFCoefficients(lfw, MOTOR_VELO_PID_LEFT)

        telemetry.addLine("Ready")
        telemetry.update()

        waitForStart()

        val inRangeRumble = Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 0.0, 200)
            .addStep(0.0, 1.0, 200)
            .addStep(1.0, 1.0, 100)
            .build()

        var targetVelocity = 0.0
        var lastDpadUp = false
        var lastDpadDown = false
        var wasAtTarget = false

        while (opModeIsActive()) {
            val dpadUp = gamepad1.dpad_up
            val dpadDown = gamepad1.dpad_down

            if (dpadUp && !lastDpadUp) {
                targetVelocity += 50.0
            }
            if (dpadDown && !lastDpadDown) {
                targetVelocity -= 50.0
            }

            targetVelocity = Range.clip(targetVelocity, 0.0, 3000.0)

            lastDpadUp = dpadUp
            lastDpadDown = dpadDown

            rfw.velocity = targetVelocity
            lfw.velocity = targetVelocity

            val rfwVelocity = rfw.velocity
            val lfwVelocity = lfw.velocity
            val actualVelocity = (abs(rfwVelocity) + abs(lfwVelocity)) / 2.0
            val velocityError = abs(targetVelocity - actualVelocity)
            val atTarget = targetVelocity > 0.0 && velocityError <= velocityTolerance

            if (atTarget && !wasAtTarget) {
                gamepad1.runRumbleEffect(inRangeRumble)
                gamepad2.runRumbleEffect(inRangeRumble)
            }
            wasAtTarget = atTarget

            telemetry.addData("targetVelocity", targetVelocity)
            telemetry.addData("rfwVelocity", rfwVelocity)
            telemetry.addData("lfwVelocity", lfwVelocity)
            telemetry.addData("actualVelocity", actualVelocity)
            telemetry.addData("velocityError", velocityError)
            telemetry.addData("atTarget", atTarget)
            telemetry.update()

            panelsTelemetry.addData("targetVelocity", targetVelocity)
            panelsTelemetry.addData("rfwVelocity", rfwVelocity)
            panelsTelemetry.addData("lfwVelocity", lfwVelocity)
            panelsTelemetry.addData("actualVelocity", actualVelocity)
            panelsTelemetry.addData("velocityError", velocityError)
            panelsTelemetry.addData("atTarget", atTarget)
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
}
