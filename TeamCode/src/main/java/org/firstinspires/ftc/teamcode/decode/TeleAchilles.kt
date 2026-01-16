package org.firstinspires.ftc.teamcode.decode

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.TriggerReader
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.ShooterConfig
import org.firstinspires.ftc.teamcode.robot.ShooterPID
import org.firstinspires.ftc.teamcode.util.PS5Keys
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

@Configurable
@TeleOp(name = "TeleAchilles")
class TeleAchilles : LinearOpMode() {
    companion object {
        @JvmField var IS_RED = false

        @JvmField var GOAL_X_BLUE = 16.0
        @JvmField var GOAL_Y_BLUE = 135.0
        @JvmField var GOAL_X_RED = 130.0
        @JvmField var GOAL_Y_RED = 135.0

        // Flywheel RPM = (a * d^2) + (b * d) + c, where d is distance to goal in Pedro units.
        @JvmField var RPM_A = 0.0
        @JvmField var RPM_B = 0.0
        @JvmField var RPM_C = 2200.0
        @JvmField var RPM_MIN = 0.0
        @JvmField var RPM_MAX = 4000.0

        // Ramp angle (deg) = (a * d^2) + (b * d) + c
        @JvmField var RAMP_A = 0.0
        @JvmField var RAMP_B = 0.0
        @JvmField var RAMP_C = 40.0
        @JvmField var RAMP_MIN_DEG = 40.0
        @JvmField var RAMP_MAX_DEG = 91.0
        @JvmField var RAMP_MIN_POS = 0.0
        @JvmField var RAMP_MAX_POS = 1.0

        @JvmField var BACK_SHOT_THRESHOLD_DEG = 90.0

        @JvmField var HEADING_KP = 2.0
        @JvmField var HEADING_TOLERANCE_DEG = 3.0
        @JvmField var RPM_TOLERANCE = 75.0
        @JvmField var RAMP_POS_TOLERANCE = 0.02

        @JvmField var FEED_POWER = 0.6
        @JvmField var RUMBLE_MS = 500
    }

    private lateinit var follower: Follower
    private lateinit var shooter: ShooterPID
    private lateinit var ffl: CRServo
    private lateinit var ffr: CRServo
    private lateinit var leftRamp: Servo
    private lateinit var rightRamp: Servo
    private lateinit var intakeLeft: DcMotorEx
    private lateinit var intakeRight: DcMotorEx
    private lateinit var panelsTelemetry: TelemetryManager

    private var shooterEnabled = false
    private var autoShootEnabled = false
    private var lastGamepad1Rb = false
    private var lastGamepad2Rb = false
    private var didRumbleReady = false

    private var targetRpm = 0.0
    private var targetRampPos = RAMP_MIN_POS

    override fun runOpMode() {
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose())
        follower.update()

        val driverGamepad = GamepadEx(gamepad1)
        val manipulatorGamepad = GamepadEx(gamepad2)
        val leftTriggerReader = TriggerReader(manipulatorGamepad, PS5Keys.Trigger.LEFT_TRIGGER.xboxTrigger)
        val rightTriggerReader = TriggerReader(manipulatorGamepad, PS5Keys.Trigger.RIGHT_TRIGGER.xboxTrigger)

        panelsTelemetry = PanelsTelemetry.telemetry

        shooter = ShooterPID(ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD, ShooterConfig.kF)
        shooter.init(hardwareMap)

        ffl = hardwareMap.get(CRServo::class.java, "feed_left").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
        ffr = hardwareMap.get(CRServo::class.java, "feed_right").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }

        leftRamp = hardwareMap.get(Servo::class.java, "left_ramp")
        rightRamp = hardwareMap.get(Servo::class.java, "right_ramp")

        intakeLeft = hardwareMap.get(DcMotorEx::class.java, "intake_left")
        intakeRight = hardwareMap.get(DcMotorEx::class.java, "intake_right")

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        follower.startTeleopDrive(true)

        while (opModeIsActive()) {
            follower.update()

            val gamepad2Rb = gamepad2.right_bumper
            val gamepad1Rb = gamepad1.right_bumper
            val gamepad2RbPressed = gamepad2Rb && !lastGamepad2Rb
            val gamepad1RbPressed = gamepad1Rb && !lastGamepad1Rb
            lastGamepad2Rb = gamepad2Rb
            lastGamepad1Rb = gamepad1Rb

            val pose = follower.pose
            val goal = getGoalPose()
            val distance = hypot(goal.x - pose.x, goal.y - pose.y)
            val targetRampDeg = computeRampDeg(distance)

            if (gamepad2RbPressed) {
                shooterEnabled = true
                autoShootEnabled = false
                didRumbleReady = false
                targetRpm = computeTargetRpm(distance)
                targetRampPos = rampDegToPos(targetRampDeg)
                shooter.setTargetRpm(targetRpm)
                leftRamp.position = targetRampPos
                rightRamp.position = targetRampPos
            }

            if (gamepad1RbPressed && shooterEnabled) {
                autoShootEnabled = true
            }

            if (!shooterEnabled) {
                shooter.setTargetRpm(0.0)
                leftRamp.position = RAMP_MIN_POS
                rightRamp.position = RAMP_MIN_POS
            }

            shooter.update()

            val shooterReady = isShooterReady()
            if (shooterEnabled && shooterReady && !didRumbleReady) {
                gamepad1.rumble(RUMBLE_MS)
                didRumbleReady = true
            }

            if (autoShootEnabled) {
                val pose = follower.pose
                val goal = getGoalPose()
                val targetHeading = computeTargetHeadingRad(pose, goal)
                val headingError = angleWrap(targetHeading - pose.heading)
                val turn = Range.clip(headingError * HEADING_KP, -1.0, 1.0)

                follower.setTeleOpDrive(0.0, 0.0, turn, false)

                val onHeading = abs(Math.toDegrees(headingError)) <= HEADING_TOLERANCE_DEG
                if (onHeading && shooterReady) {
                    ffl.power = FEED_POWER
                    ffr.power = FEED_POWER
                } else {
                    ffl.power = 0.0
                    ffr.power = 0.0
                }
            } else {
                follower.setTeleOpDrive(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble(),
                    false // Field centric
                )
                ffl.power = 0.0
                ffr.power = 0.0
            }
            if (manipulatorGamepad.getTrigger(PS5Keys.Trigger.RIGHT_TRIGGER.xboxTrigger) >= 0.15){
                intakeLeft.power = .75
                intakeRight.power = .75
            }else if (manipulatorGamepad.getTrigger(PS5Keys.Trigger.LEFT_TRIGGER.xboxTrigger) >= 0.15){
                intakeLeft.power = -.75
                intakeRight.power = -.75
            }else {
                intakeLeft.power = 0.0
                intakeRight.power = 0.0
            }

            telemetry.addData("Shooter Enabled", shooterEnabled)
            telemetry.addData("Auto Shoot", autoShootEnabled)
            telemetry.addData("Target RPM", "%.0f", targetRpm)
            telemetry.addData("Ramp Pos", "%.3f", targetRampPos)
            telemetry.update()

            panelsTelemetry.debug("target_rpm", targetRpm)
            panelsTelemetry.debug("flywheel_rpm", shooter.getFlywheelRpm())
            panelsTelemetry.debug("distance_to_goal", distance)
            panelsTelemetry.debug("target_ramp_deg", targetRampDeg)
            panelsTelemetry.debug("target_ramp_pos", targetRampPos)
            panelsTelemetry.debug("left_ramp_pos", leftRamp.position)
            panelsTelemetry.debug("right_ramp_pos", rightRamp.position)
            panelsTelemetry.update(telemetry)
            idle()
        }
    }

    private fun getGoalPose(): Pose {
        return if (IS_RED) {
            Pose(GOAL_X_RED, GOAL_Y_RED, 0.0)
        } else {
            Pose(GOAL_X_BLUE, GOAL_Y_BLUE, 0.0)
        }
    }

    private fun computeTargetRpm(distance: Double): Double {
        val rpm = (RPM_A * distance * distance) + (RPM_B * distance) + RPM_C
        return Range.clip(rpm, RPM_MIN, RPM_MAX)
    }

    private fun computeRampDeg(distance: Double): Double {
        val deg = (RAMP_A * distance * distance) + (RAMP_B * distance) + RAMP_C
        return Range.clip(deg, RAMP_MIN_DEG, RAMP_MAX_DEG)
    }

    private fun rampDegToPos(deg: Double): Double {
        val t = (deg - RAMP_MIN_DEG) / (RAMP_MAX_DEG - RAMP_MIN_DEG)
        return Range.clip(RAMP_MIN_POS + t * (RAMP_MAX_POS - RAMP_MIN_POS), RAMP_MIN_POS, RAMP_MAX_POS)
    }

    private fun computeTargetHeadingRad(pose: Pose, goal: Pose): Double {
        val headingToGoal = atan2(goal.y - pose.y, goal.x - pose.x)
        val distance = hypot(goal.x - pose.x, goal.y - pose.y)
        val rampDeg = computeRampDeg(distance)
        return if (rampDeg >= BACK_SHOT_THRESHOLD_DEG) {
            angleWrap(headingToGoal + Math.PI)
        } else {
            angleWrap(headingToGoal)
        }
    }

    private fun angleWrap(angle: Double): Double {
        var a = angle
        while (a > Math.PI) a -= 2.0 * Math.PI
        while (a < -Math.PI) a += 2.0 * Math.PI
        return a
    }

    private fun isShooterReady(): Boolean {
        val rpmError = abs(shooter.getFlywheelRpm() - targetRpm)
        val rampErrorLeft = abs(leftRamp.position - targetRampPos)
        val rampErrorRight = abs(rightRamp.position - targetRampPos)
        val rampError = max(rampErrorLeft, rampErrorRight)
        return rpmError <= RPM_TOLERANCE && rampError <= RAMP_POS_TOLERANCE
    }
}
