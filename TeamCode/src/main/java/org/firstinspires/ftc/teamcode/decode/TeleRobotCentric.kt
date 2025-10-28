package org.firstinspires.ftc.teamcode.decode

import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.TriggerReader
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.ShooterConfig
import org.firstinspires.ftc.teamcode.robot.ShooterPID
import org.firstinspires.ftc.teamcode.util.PS5Keys
import kotlin.math.max
import kotlin.math.min

/**
 * Test OpMode for complex movements including Travel Position
 */
@TeleOp(name = "Decode Robot Centric")
class TeleRobotCentric : LinearOpMode() {

//    private lateinit var follower: Follower
//    val startPose: Pose = Pose(6.5, 111.0, 0.0)

    lateinit var ffl: CRServo
    lateinit var ffr: CRServo

    lateinit var follower: Follower

    val config = ShooterConfig

    var shooter = ShooterPID(config.kP, config.kI, config.kD, config.kF)


//    val cyclePose: Pose = Pose(60.0, 98.0, Math.toRadians(270.0))
//    val cycleControlPose: Pose = Pose(65.0, 120.0, Math.toRadians(-90.0))

    override fun runOpMode() {

        val timer: ElapsedTime = ElapsedTime()

        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose())
        follower.update()

        val driveSpeedMultiplier = 1.0

        // Initialize gamepad
        val driverGamepad = GamepadEx(gamepad1)
        val manipulatorGamepad = GamepadEx(gamepad2)
        val leftTriggerReader = TriggerReader(manipulatorGamepad, PS5Keys.Trigger.LEFT_TRIGGER.xboxTrigger)
        val rightTriggerReader = TriggerReader(manipulatorGamepad, PS5Keys.Trigger.RIGHT_TRIGGER.xboxTrigger)

        // Initialize drive motors

        ffl = hardwareMap.get(CRServo::class.java, "feed_left").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }

        ffr = hardwareMap.get(CRServo::class.java, "feed_right").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }

        shooter = ShooterPID(config.kP, config.kI, config.kD, config.kF)
        shooter.init(hardwareMap)

        val frontLeft = Motor(hardwareMap, "left_front", Motor.GoBILDA.RPM_1150).apply {
            setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        }
        val frontRight = Motor(hardwareMap, "right_front", Motor.GoBILDA.RPM_1150).apply {
            setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        }
        val backLeft = Motor(hardwareMap, "left_back", Motor.GoBILDA.RPM_1150).apply {
            setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        }
        val backRight = Motor(hardwareMap, "right_back", Motor.GoBILDA.RPM_1150).apply {
            setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        }
        val driveTrain = MecanumDrive(frontLeft, frontRight, backLeft, backRight)

        var manualOverride = false
        var resetEncoders = false

        telemetry.addData("Status", "Initialized")
        telemetry.addData("Controls", "Y button: Travel Position")
        telemetry.update()

        waitForStart()

//        follower.startTeleopDrive(true)

        var shooterSpeed = 2750.0

        while (opModeIsActive()) {
            shooter.update()
            follower.update()


            // Read gamepad inputs
            driverGamepad.readButtons()
            manipulatorGamepad.readButtons()
            leftTriggerReader.readValue()
            rightTriggerReader.readValue()

            shooter.update()

            if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_UP.xboxButton)){
                shooterSpeed = min(3000.0, shooterSpeed + 250.0)
                shooter.setTargetRpm(shooterSpeed)
                shooter.update()
            }else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_DOWN.xboxButton)) {
                shooterSpeed = max(1000.0, shooterSpeed - 250.0)
                shooter.setTargetRpm(shooterSpeed)
                shooter.update()
            }else if (manipulatorGamepad.wasJustPressed(PS5Keys.Button.DPAD_LEFT.xboxButton)) {
                shooter.setTargetRpm(0.0)
            }
            shooter.update()
            if (manipulatorGamepad.isDown((PS5Keys.Button.RIGHT_BUMPER.xboxButton))){
                ffl.power = 0.8
                ffr.power = 0.8
            }else if (manipulatorGamepad.isDown((PS5Keys.Button.LEFT_BUMPER.xboxButton))) {
                ffl.power = -0.5
                ffr.power = -0.5
            }else {
                ffl.power = 0.0
                ffr.power = 0.0
            }


//            driveTrain.driveRobotCentric(
//                -driverGamepad.leftX * driveSpeedMultiplier,
//                driverGamepad.leftY * driveSpeedMultiplier,
//                -driverGamepad.rightX * driveSpeedMultiplier,
//            )
//

            follower.setTeleOpDrive(
                gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                true
            )
            follower.update()

            shooter.update()

            // Display telemetry
            telemetry.addData("Status", "Running")
            telemetry.addData("Motor Speed RPM:", shooter.flywheel())
//            telemetry.addData("X", follower.pose.x)
//            telemetry.addData("Y", follower.pose.y)
//            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.pose.heading))
            telemetry.update()

        }
    }
}
