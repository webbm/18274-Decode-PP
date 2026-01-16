package org.firstinspires.ftc.teamcode.tuners

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.ShooterConfig
import org.firstinspires.ftc.teamcode.robot.ShooterPID
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.BufferedWriter
import java.io.FileWriter
import kotlin.math.hypot
import kotlin.math.max

@TeleOp(name = "Shooter Equation Finder", group = "Tuning")
@Configurable
class ShooterEquationFinder : OpMode() {

    companion object {
        @JvmField var START_X = 0.0
        @JvmField var START_Y = 0.0
        @JvmField var START_HEADING_DEG = 0.0
        @JvmField var USE_MANUAL_DISTANCE = false
        @JvmField var GOAL_X = 0.0
        @JvmField var GOAL_Y = 0.0
        @JvmField var MANUAL_DISTANCE_IN = 0.0
        @JvmField var RPM_STEP = 150.0
        @JvmField var FEED_POWER = 1.0
        @JvmField var INTAKE_POWER = 0.15
    }

    private lateinit var follower: Follower
    private lateinit var shooterPID: ShooterPID
    private lateinit var intake: DcMotorEx
    private lateinit var ffl: CRServo
    private lateinit var ffr: CRServo
    private lateinit var leftRamp: Servo
    private lateinit var rightRamp: Servo
    private lateinit var panelsTelemetry: TelemetryManager

    private var targetRpm = 2200.0

    private var lastA = false
    private var lastY = false
    private var lastX = false

    private val savedShots = mutableListOf<String>()
    private val logFile by lazy { AppUtil.getInstance().getSettingsFile("ShooterEquationFinder.csv") }

    override fun init() {
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)))

        shooterPID = ShooterPID(ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD, ShooterConfig.kF)
        shooterPID.init(hardwareMap)

        intake = hardwareMap.get(DcMotorEx::class.java, "intake")
        intake.direction = DcMotorSimple.Direction.REVERSE

        ffl = hardwareMap.get(CRServo::class.java, "feed_left")
        ffr = hardwareMap.get(CRServo::class.java, "feed_right")
        ffl.direction = DcMotorSimple.Direction.REVERSE
        leftRamp = hardwareMap.get(Servo::class.java, "left_ramp")
        rightRamp = hardwareMap.get(Servo::class.java, "right_ramp")
        panelsTelemetry = PanelsTelemetry.telemetry

        if (!logFile.exists()) {
            appendShotLog("x,y,heading_deg,distance_to_goal_in,flywheel_rpm,left_ramp_pos,right_ramp_pos")
        }

        telemetry.addData("Status", "Initialized")
        telemetry.update()
    }

    override fun loop() {
        follower.update()

        val aPressed = gamepad2.a && !lastA
        val yPressed = gamepad2.y && !lastY
        val xPressed = gamepad2.x && !lastX
        lastA = gamepad2.a
        lastY = gamepad2.y
        lastX = gamepad2.x

        if (aPressed) {
            targetRpm = max(0.0, targetRpm - RPM_STEP)
        }
        if (yPressed) {
            targetRpm += RPM_STEP
        }

        shooterPID.setTargetRpm(targetRpm)
        shooterPID.update()

        val feed = gamepad2.right_trigger > 0.5
        if (feed) {
            ffl.power = FEED_POWER
            ffr.power = FEED_POWER
            intake.power = INTAKE_POWER
        } else {
            ffl.power = 0.0
            ffr.power = 0.0
            intake.power = 0.0
        }

        val pose = follower.pose
        val distanceToGoal = if (USE_MANUAL_DISTANCE) {
            MANUAL_DISTANCE_IN
        } else {
            hypot(GOAL_X - pose.x, GOAL_Y - pose.y)
        }

        if (xPressed) {
            val flywheelRpm = shooterPID.getFlywheelRpm()
            appendShotLog(
                String.format(
                    "%.2f,%.2f,%.1f,%.2f,%.0f,%.4f,%.4f",
                    pose.x,
                    pose.y,
                    Math.toDegrees(pose.heading),
                    distanceToGoal,
                    flywheelRpm,
                    leftRamp.position,
                    rightRamp.position
                )
            )
            val record = String.format(
                "X %.2f Y %.2f H %.1f D %.2f RPM %.0f",
                pose.x,
                pose.y,
                Math.toDegrees(pose.heading),
                distanceToGoal,
                targetRpm
            )
            savedShots.add(0, record)
            if (savedShots.size > 6) {
                savedShots.removeAt(savedShots.size - 1)
            }
        }

        telemetry.addData("Pose X", "%.2f", pose.x)
        telemetry.addData("Pose Y", "%.2f", pose.y)
        telemetry.addData("Heading", "%.1f", Math.toDegrees(pose.heading))
        telemetry.addData("Distance to Goal (in)", "%.2f", distanceToGoal)
        telemetry.addData("Target RPM", "%.0f", targetRpm)
        telemetry.addData("Feed (RT)", feed)
        telemetry.addLine("A: -RPM, Y: +RPM, X: Save point")
        for (shot in savedShots) {
            telemetry.addLine(shot)
        }
        telemetry.update()

        panelsTelemetry.debug("pose_x", pose.x)
        panelsTelemetry.debug("pose_y", pose.y)
        panelsTelemetry.debug("heading_deg", Math.toDegrees(pose.heading))
        panelsTelemetry.debug("distance_to_goal_in", distanceToGoal)
        panelsTelemetry.debug("target_rpm", targetRpm)
        panelsTelemetry.debug("flywheel_rpm", shooterPID.getFlywheelRpm())
        panelsTelemetry.debug("left_ramp_pos", leftRamp.position)
        panelsTelemetry.debug("right_ramp_pos", rightRamp.position)
        panelsTelemetry.update(telemetry)
    }

    private fun appendShotLog(line: String) {
        try {
            BufferedWriter(FileWriter(logFile, true)).use { writer ->
                writer.append(line)
                writer.newLine()
            }
        } catch (e: Exception) {
            RobotLog.ee("ShooterEquationFinder", e, "Failed to write %s", logFile.absolutePath)
        }
    }
}
