package org.firstinspires.ftc.teamcode.decode

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.ShooterConfig
import org.firstinspires.ftc.teamcode.robot.ShooterPID
import org.firstinspires.ftc.teamcode.util.CurrentLogger
import org.firstinspires.ftc.teamcode.util.PS5Keys
import kotlin.math.hypot

@Configurable
@TeleOp(name = "TeleAchilles")
class TeleAchilles : LinearOpMode() {
   /* companion object {
        @JvmField var IS_RED = false

        @JvmField var GOAL_X_BLUE = 16.0
        @JvmField var GOAL_Y_BLUE = 135.0
        @JvmField var GOAL_X_RED = 130.0
        @JvmField var GOAL_Y_RED = 135.0

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

        @JvmField var FEED_POWER = 0.6
    }
*/
    private lateinit var follower: Follower
    private lateinit var ffl: CRServo
    private lateinit var ffr: CRServo
//    private lateinit var leftRamp: Servo
//    private lateinit var rightRamp: Servo
    private lateinit var intakeLeft: DcMotorEx
    private lateinit var intakeRight: DcMotorEx

    private lateinit var leftFlywheel: DcMotorEx

    private lateinit var rightFlywheel: DcMotorEx

    private lateinit var panelsTelemetry: TelemetryManager
    private lateinit var lynxModules: List<LynxModule>
    private lateinit var currentLogger: CurrentLogger


    override fun runOpMode() {
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose())
        follower.update()

        val manipulatorGamepad = GamepadEx(gamepad2)

        panelsTelemetry = PanelsTelemetry.telemetry


        ffl = hardwareMap.get(CRServo::class.java, "feed_left").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
        ffr = hardwareMap.get(CRServo::class.java, "feed_right").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }
//
//        leftRamp = hardwareMap.get(Servo::class.java, "left_ramp")
//        rightRamp = hardwareMap.get(Servo::class.java, "right_ramp")

        intakeLeft = hardwareMap.get(DcMotorEx::class.java, "intake_left")
        intakeRight = hardwareMap.get(DcMotorEx::class.java, "intake_right").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }

        leftFlywheel = hardwareMap.get(DcMotorEx::class.java, "left_flywheel").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }
        rightFlywheel = hardwareMap.get(DcMotorEx::class.java, "right_flywheel").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }

        lynxModules = hardwareMap.getAll(LynxModule::class.java)
        currentLogger = CurrentLogger(hardwareMap, logIntervalMs = 100L)

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        follower.startTeleopDrive(true)
        currentLogger.start()

        while (opModeIsActive()) {
            follower.update()
            manipulatorGamepad.readButtons()

            if (manipulatorGamepad.getTrigger(PS5Keys.Trigger.RIGHT_TRIGGER.xboxTrigger) >= 0.1){
                leftFlywheel.power = manipulatorGamepad.getTrigger(PS5Keys.Trigger.RIGHT_TRIGGER.xboxTrigger)
                rightFlywheel.power = manipulatorGamepad.getTrigger(PS5Keys.Trigger.RIGHT_TRIGGER.xboxTrigger)
            }else {
                leftFlywheel.power = 0.0
                rightFlywheel.power = 0.0
            }

//            shooter.setTargetRpm(0.0)
//            shooter.update()
//
//            if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_UP.xboxButton)) {
//                targetRpm = Range.clip(targetRpm + 50.0, RPM_MIN, RPM_MAX)
//            } else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_DOWN.xboxButton)) {
//                targetRpm = Range.clip(targetRpm - 50.0, RPM_MIN, RPM_MAX)
//            } else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.CROSS.xboxButton)) {
//                if (targetRpm > 0.0) {
//                    lastNonZeroRpm = targetRpm
//                }
//                targetRpm = 0.0
//            } else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.TRIANGLE.xboxButton)) {
//                if (lastNonZeroRpm > 0.0) {
//                    targetRpm = lastNonZeroRpm
//                }
//            }
//
//            if (targetRpm > 0.0) {
//                lastNonZeroRpm = targetRpm
//            }

            val pose = follower.pose
//            val goal = getGoalPose()
//            val distance = hypot(goal.x - pose.x, goal.y - pose.y)
//            val targetRampDeg = computeRampDeg(distance)

            follower.setTeleOpDrive(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                true // Field centric
            )

           ffr.power = manipulatorGamepad.leftX
           ffl.power = manipulatorGamepad.leftX

            val feedPower = (ffl.power + ffr.power) / 2.0

            if (manipulatorGamepad.getButton(PS5Keys.Button.RIGHT_BUMPER.xboxButton)){
                intakeLeft.power = .75
                intakeRight.power = .75
            }else if (manipulatorGamepad.getButton(PS5Keys.Button.LEFT_BUMPER.xboxButton)){
                intakeLeft.power = -.75
                intakeRight.power = -.75
            }else {
                intakeLeft.power = 0.0
                intakeRight.power = 0.0
            }
            val intakePower = (intakeLeft.power + intakeRight.power) / 2.0

            var totalCurrent = 0.0
            for (module in lynxModules) {
                totalCurrent += module.getCurrent(CurrentUnit.AMPS)
            }

            telemetry.addData("Feed On", feedPower != 0.0)
            telemetry.addData("Intake Power", "%.2f", intakePower)
            telemetry.addData("Flywheel power", "%.2f", (leftFlywheel.power + rightFlywheel.power)/2)
            telemetry.addData("Total Current (A)", "%.2f", totalCurrent)
            telemetry.update()

//            panelsTelemetry.debug("target_rpm", targetRpm)
//            panelsTelemetry.debug("saved_rpm", lastNonZeroRpm)
//            panelsTelemetry.debug("flywheel_rpm", shooter.getFlywheelRpm())
//            panelsTelemetry.debug("feed_on", feedPower != 0.0)
//            panelsTelemetry.debug("intake_power", intakePower)
//            panelsTelemetry.debug("total_current_amps", totalCurrent)
//            panelsTelemetry.debug("distance_to_goal", distance)
//            panelsTelemetry.debug("target_ramp_deg", targetRampDeg)
//            panelsTelemetry.debug("target_ramp_pos", targetRampPos)
//            panelsTelemetry.debug("left_ramp_pos", leftRamp.position)
//            panelsTelemetry.debug("right_ramp_pos", rightRamp.position)
//            panelsTelemetry.update(telemetry)
            currentLogger.update()
            idle()
        }
    }

//    private fun getGoalPose(): Pose {
//        return if (IS_RED) {
//            Pose(GOAL_X_RED, GOAL_Y_RED, 0.0)
//        } else {
//            Pose(GOAL_X_BLUE, GOAL_Y_BLUE, 0.0)
//        }
//    }
//
//    private fun computeRampDeg(distance: Double): Double {
//        val deg = (RAMP_A * distance * distance) + (RAMP_B * distance) + RAMP_C
//        return Range.clip(deg, RAMP_MIN_DEG, RAMP_MAX_DEG)
//    }
//
//    private fun rampDegToPos(deg: Double): Double {
//        val t = (deg - RAMP_MIN_DEG) / (RAMP_MAX_DEG - RAMP_MIN_DEG)
//        return Range.clip(RAMP_MIN_POS + t * (RAMP_MAX_POS - RAMP_MIN_POS), RAMP_MIN_POS, RAMP_MAX_POS)
//    }

}
