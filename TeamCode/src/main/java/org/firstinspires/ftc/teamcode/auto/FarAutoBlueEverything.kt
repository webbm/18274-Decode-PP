package org.firstinspires.ftc.teamcode.auto

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.IntakeConfig
import org.firstinspires.ftc.teamcode.robot.IntakePID
import org.firstinspires.ftc.teamcode.robot.ShooterConfig
import org.firstinspires.ftc.teamcode.robot.ShooterPID

@Autonomous(name = "FarAutoBlueEverything", group = "Blue")
@Configurable
class FarAutoBlueEverything : OpMode() {

    companion object {
        @JvmField var PRELOAD_RPM = 2300.0
        @JvmField var CYCLE_RPM = 2200.0
        @JvmField var INTAKE_RPM = 2000.0

        @JvmField var SPINUP_TIME_SEC = 0.75
        @JvmField var FEED_TIME_SEC = 1.5
        @JvmField var INTAKE_TIME_SEC = 1.25

        @JvmField var FEED_POWER = 1.0
    }

    private lateinit var panelsTelemetry: TelemetryManager
    private lateinit var follower: Follower
    private var pathState = 0
    private lateinit var paths: Paths

    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    private lateinit var shooterPID: ShooterPID
    private lateinit var intakePID: IntakePID
    private lateinit var ffl: CRServo
    private lateinit var ffr: CRServo

    class Paths(follower: Follower) {
        val scorePos: PathChain
        val pickup1Prep: PathChain
        val pickup1Ex: PathChain
        val scorePos2: PathChain
        val pickup2prep: PathChain
        val pickup2Ex: PathChain
        val scorePos3: PathChain
        val pickup3Prep: PathChain
        val pickup3Ex: PathChain
        val scorePos4: PathChain
        val intakePrep: PathChain
        val intake: PathChain
        val park: PathChain

        init {
            scorePos = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(57.200, 9.100),
                    Pose(60.000, 24.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(112.0))
                .build()

            pickup1Prep = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(60.000, 24.000),
                    Pose(42.000, 36.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(112.0), Math.toRadians(0.0))
                .build()

            pickup1Ex = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(42.000, 36.000),
                    Pose(10.000, 36.000)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(0.0))
                .build()

            scorePos2 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(10.000, 36.000),
                    Pose(60.000, 24.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(112.0))
                .build()

            pickup2prep = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(60.000, 24.000),
                    Pose(42.000, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(112.0), Math.toRadians(0.0))
                .build()

            pickup2Ex = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(42.000, 60.000),
                    Pose(10.000, 58.500)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(0.0))
                .build()

            scorePos3 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(10.000, 58.500),
                    Pose(55.500, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(127.0))
                .build()

            pickup3Prep = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(55.500, 84.000),
                    Pose(44.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(127.0), Math.toRadians(0.0))
                .build()

            pickup3Ex = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(44.000, 84.000),
                    Pose(16.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
                .build()

            scorePos4 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(16.000, 84.000),
                    Pose(48.000, 95.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(127.0))
                .build()

            intakePrep = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(48.000, 95.000),
                    Pose(31.750, 9.750)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(127.0), Math.toRadians(0.0))
                .build()

            intake = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(31.750, 9.750),
                    Pose(10.000, 9.750)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
                .build()

            park = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(10.000, 9.750),
                    Pose(36.000, 12.000)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(0.0))
                .build()
        }
    }

    private fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }

    private fun setFeedPower(power: Double) {
        ffl.power = power
        ffr.power = power
    }

    private fun startShooting(rpm: Double) {
        shooterPID.setTargetRpm(rpm)
        actionTimer.resetTimer()
    }

    private fun stopShooting() {
        shooterPID.setTargetRpm(0.0)
    }

    private fun startFeeding() {
        setFeedPower(FEED_POWER)
        actionTimer.resetTimer()
    }

    private fun stopFeeding() {
        setFeedPower(0.0)
    }

    private fun startIntake() {
        intakePID.setTargetRpm(INTAKE_RPM)
        actionTimer.resetTimer()
    }

    private fun stopIntake() {
        intakePID.setTargetRpm(0.0)
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(paths.scorePos, true)
                setPathState(1)
            }
            1 -> {
                if (!follower.isBusy) {
                    startShooting(PRELOAD_RPM)
                    setPathState(2)
                }
            }
            2 -> {
                if (actionTimer.elapsedTimeSeconds > SPINUP_TIME_SEC) {
                    startFeeding()
                    setPathState(3)
                }
            }
            3 -> {
                if (actionTimer.elapsedTimeSeconds > FEED_TIME_SEC) {
                    stopFeeding()
                    stopShooting()
                    follower.followPath(paths.pickup1Prep, true)
                    setPathState(4)
                }
            }
            4 -> {
                if (!follower.isBusy) {
                    follower.followPath(paths.pickup1Ex, true)
                    startIntake()
                    setPathState(5)
                }
            }
            5 -> {
                if (!follower.isBusy || actionTimer.elapsedTimeSeconds > INTAKE_TIME_SEC) {
                    stopIntake()
                    follower.followPath(paths.scorePos2, true)
                    setPathState(6)
                }
            }
            6 -> {
                if (!follower.isBusy) {
                    startShooting(CYCLE_RPM)
                    setPathState(7)
                }
            }
            7 -> {
                if (actionTimer.elapsedTimeSeconds > SPINUP_TIME_SEC) {
                    startFeeding()
                    setPathState(8)
                }
            }
            8 -> {
                if (actionTimer.elapsedTimeSeconds > FEED_TIME_SEC) {
                    stopFeeding()
                    stopShooting()
                    follower.followPath(paths.pickup2prep, true)
                    setPathState(9)
                }
            }
            9 -> {
                if (!follower.isBusy) {
                    follower.followPath(paths.pickup2Ex, true)
                    startIntake()
                    setPathState(10)
                }
            }
            10 -> {
                if (!follower.isBusy || actionTimer.elapsedTimeSeconds > INTAKE_TIME_SEC) {
                    stopIntake()
                    follower.followPath(paths.scorePos3, true)
                    setPathState(11)
                }
            }
            11 -> {
                if (!follower.isBusy) {
                    startShooting(CYCLE_RPM)
                    setPathState(12)
                }
            }
            12 -> {
                if (actionTimer.elapsedTimeSeconds > SPINUP_TIME_SEC) {
                    startFeeding()
                    setPathState(13)
                }
            }
            13 -> {
                if (actionTimer.elapsedTimeSeconds > FEED_TIME_SEC) {
                    stopFeeding()
                    stopShooting()
                    follower.followPath(paths.pickup3Prep, true)
                    setPathState(14)
                }
            }
            14 -> {
                if (!follower.isBusy) {
                    follower.followPath(paths.pickup3Ex, true)
                    startIntake()
                    setPathState(15)
                }
            }
            15 -> {
                if (!follower.isBusy || actionTimer.elapsedTimeSeconds > INTAKE_TIME_SEC) {
                    stopIntake()
                    follower.followPath(paths.scorePos4, true)
                    setPathState(16)
                }
            }
            16 -> {
                if (!follower.isBusy) {
                    startShooting(CYCLE_RPM)
                    setPathState(17)
                }
            }
            17 -> {
                if (actionTimer.elapsedTimeSeconds > SPINUP_TIME_SEC) {
                    startFeeding()
                    setPathState(18)
                }
            }
            18 -> {
                if (actionTimer.elapsedTimeSeconds > FEED_TIME_SEC) {
                    stopFeeding()
                    stopShooting()
                    follower.followPath(paths.intakePrep, true)
                    setPathState(19)
                }
            }
            19 -> {
                if (!follower.isBusy) {
                    follower.followPath(paths.intake, true)
                    startIntake()
                    setPathState(20)
                }
            }
            20 -> {
                if (!follower.isBusy || actionTimer.elapsedTimeSeconds > INTAKE_TIME_SEC) {
                    stopIntake()
                    follower.followPath(paths.park, true)
                    setPathState(21)
                }
            }
            21 -> {
                if (!follower.isBusy) {
                    shooterPID.stop()
                    intakePID.stop()
                    setPathState(-1)
                }
            }
            else -> {
                // Idle
            }
        }
    }

    override fun init() {
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()

        actionTimer.resetTimer()
        opmodeTimer.resetTimer()

        panelsTelemetry = PanelsTelemetry.telemetry

        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose(57.200, 9.100, Math.toRadians(90.0)))

        paths = Paths(follower)

        shooterPID = ShooterPID(ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD, ShooterConfig.kF)
        shooterPID.init(hardwareMap)

        intakePID = IntakePID(IntakeConfig.kP, IntakeConfig.kI, IntakeConfig.kD, IntakeConfig.kF)
        intakePID.init(hardwareMap)

        ffl = hardwareMap.get(CRServo::class.java, "feed_left").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
        ffr = hardwareMap.get(CRServo::class.java, "feed_right").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }

        panelsTelemetry.debug("Status", "Initialized")
        panelsTelemetry.update(telemetry)
    }

    override fun loop() {
        follower.update()
        shooterPID.update()
        intakePID.update()

        autonomousPathUpdate()

        panelsTelemetry.debug("Path State", pathState)
        panelsTelemetry.debug("X", follower.pose.x)
        panelsTelemetry.debug("Y", follower.pose.y)
        panelsTelemetry.debug("Heading", follower.pose.heading)
        panelsTelemetry.update(telemetry)
    }
}
