package org.firstinspires.ftc.teamcode.decode

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.FlywheelShooter
import org.firstinspires.ftc.teamcode.util.PS5Keys
import kotlin.math.abs

@TeleOp(name = "LTTeleOp")
class TeleOpShooterPidRobotCentric : LinearOpMode() {
    companion object {
        @JvmField var SHOOTER_RPM_INITIAL = 1225.0
        @JvmField var SHOOTER_RPM_STEP = 50.0
        @JvmField var SHOOTER_RPM_MIN = 0.0
        @JvmField var SHOOTER_RPM_MAX = 4000.0
        @JvmField var INTAKE_POWER = 1.0
        @JvmField var FEED_POWER = 1.0
        @JvmField var RAMP_POWER = 1.0
        @JvmField var INTAKE_VOLT_DROP = 0.25
        @JvmField var INTAKE_VOLT_RECOVER = 0.15
        @JvmField var INTAKE_DROP_DEBOUNCE_MS = 250L
    }

    private lateinit var follower: Follower
    private lateinit var flywheelShooter: FlywheelShooter

    private lateinit var intakeLeft: DcMotorEx
    private lateinit var intakeRight: DcMotorEx
    private lateinit var feedLeft: CRServo
    private lateinit var feedRight: CRServo
    private lateinit var voltageSensors: List<VoltageSensor>

    override fun runOpMode() {
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose())
        follower.update()

        val manipulatorGamepad = GamepadEx(gamepad2)
        val driverGamepad = GamepadEx(gamepad1)

        flywheelShooter = FlywheelShooter(
            rightMotorName = "right_flywheel",
            leftMotorName = "left_flywheel",
            rightDirection = DcMotorSimple.Direction.REVERSE,
            leftDirection = DcMotorSimple.Direction.FORWARD
        )
        flywheelShooter.init(hardwareMap)

        intakeLeft = hardwareMap.get(DcMotorEx::class.java, "intake_left")
        intakeRight = hardwareMap.get(DcMotorEx::class.java, "intake_right").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }

        feedLeft = hardwareMap.get(CRServo::class.java, "feed_left").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
        feedRight = hardwareMap.get(CRServo::class.java, "feed_right").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }

        voltageSensors = hardwareMap.voltageSensor.toList()

        telemetry.addData("Status", "Initialized")
        telemetry.addLine("Shooter: Triangle toggle, Dpad Up/Down +/-RPM")
        telemetry.addLine("Intake: RT in, LT out")
        telemetry.addLine("Feed: RB forward, LB reverse")
        telemetry.addLine("Ramp: Dpad Up forward, Dpad Down reverse")
        telemetry.update()

        waitForStart()

        follower.startTeleopDrive(true)

        var targetRpm = SHOOTER_RPM_INITIAL
        var shooterEnabled = false
        var ballCount = 0
        var intakeBaselineVoltage = 0.0
        var inVoltageDrop = false
        var lastDropMs = 0L
        val timer = ElapsedTime()

        while (opModeIsActive()) {
            follower.update()

            manipulatorGamepad.readButtons()
            driverGamepad.readButtons()

            follower.setTeleOpDrive(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                false // Robot centric
            )

            if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.TRIANGLE.xboxButton)) {
                shooterEnabled = !shooterEnabled
                if (!shooterEnabled) {
                    flywheelShooter.setVelocity(0.0)
                }
            }

            if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_DOWN.xboxButton)) {
                targetRpm = 1000.0
            } else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_RIGHT.xboxButton)) {
                targetRpm = 1050.0
            } else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_LEFT.xboxButton)) {
                targetRpm = 1100.0
            } else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_UP.xboxButton)) {
                targetRpm = 1225.0
            }

            if (shooterEnabled) {
                flywheelShooter.setVelocity(targetRpm)
            } else {
                flywheelShooter.setVelocity(0.0)
            }

            val intakePower = when {
                gamepad2.right_trigger > 0.1 -> INTAKE_POWER
                gamepad2.left_trigger > 0.1 -> -INTAKE_POWER
                else -> 0.0
            }
            intakeLeft.power = intakePower
            intakeRight.power = intakePower

            val intakeActive = abs(intakePower) > 0.05
            val avgVoltage = if (voltageSensors.isNotEmpty()) {
                voltageSensors.sumOf { it.voltage } / voltageSensors.size
            } else {
                0.0
            }

            if (intakeActive && intakeBaselineVoltage <= 0.0) {
                intakeBaselineVoltage = avgVoltage
            }
            if (!intakeActive) {
                intakeBaselineVoltage = 0.0
                inVoltageDrop = false
            } else if (intakeBaselineVoltage > 0.0) {
                val nowMs = timer.milliseconds().toLong()
                val dropThreshold = intakeBaselineVoltage - INTAKE_VOLT_DROP
                val recoverThreshold = intakeBaselineVoltage - INTAKE_VOLT_RECOVER

                if (!inVoltageDrop && avgVoltage < dropThreshold && (nowMs - lastDropMs) > INTAKE_DROP_DEBOUNCE_MS) {
                    ballCount += 1
                    inVoltageDrop = true
                    lastDropMs = nowMs
                } else if (inVoltageDrop && avgVoltage > recoverThreshold) {
                    inVoltageDrop = false
                    intakeBaselineVoltage = avgVoltage
                }
            }

            if (driverGamepad.wasJustPressed(PS5Keys.Button.CROSS.xboxButton)) {
                follower.pose = Pose()
            }

            val feedPower = when {
                gamepad2.right_bumper -> FEED_POWER
                gamepad2.left_bumper -> -FEED_POWER
                else -> 0.0
            }
            feedLeft.power = feedPower
            feedRight.power = feedPower


            telemetry.addData("Shooter Enabled", shooterEnabled)
            telemetry.addData("Target RPM", "%.0f", targetRpm)
            telemetry.addData("Flywheel Velocity", "%.0f", flywheelShooter.getVelocity())
            telemetry.addData("Intake Power", "%.2f", intakePower)
            telemetry.addData("Intake Balls", ballCount)
            telemetry.addData("Intake Voltage", "%.2f", avgVoltage)
            telemetry.addData("Feed Power", "%.2f", feedPower)
            telemetry.update()

            idle()
        }
    }

}
