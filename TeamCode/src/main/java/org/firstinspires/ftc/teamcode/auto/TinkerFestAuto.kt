package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveByGyro_Linear
import org.firstinspires.ftc.teamcode.robot.ShooterConfig
import org.firstinspires.ftc.teamcode.robot.ShooterPID
import kotlin.math.abs
import kotlin.math.max

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name = "Tinker Auto")
//@Disabled
class TinkerFestAuto : LinearOpMode() {
    /* Declare OpMode members. */
    lateinit var leftDrive: DcMotor
    lateinit var leftBackDrive: DcMotor
    lateinit var rightDrive: DcMotor
    lateinit var rightBackDrive: DcMotor

    lateinit var ffl: CRServo
    lateinit var ffr: CRServo

    lateinit var intake: DcMotorEx

    val config = ShooterConfig

    var shooter = ShooterPID(config.kP, config.kI, config.kD, config.kF)

    lateinit var imu: IMU

    var targetHeading = 0.0
    var driveSpeed = 0.0
    var turnSpeed = 0.0
    var leftSpeed = 0.0
    var rightSpeed = 0.0

    var headingError = 0.0

    val P_TURN_GAIN = 0.02
    val P_DRIVE_GAIN = 0.03

    private val runtime = ElapsedTime()


    override fun runOpMode() {
        // Initialize the drive system variables.

        leftDrive = hardwareMap.get(DcMotor::class.java, "left_front").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            direction = DcMotorSimple.Direction.FORWARD
        }
        leftBackDrive = hardwareMap.get(DcMotor::class.java, "left_back").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            direction = DcMotorSimple.Direction.FORWARD
        }
        rightDrive = hardwareMap.get(DcMotor::class.java, "right_front").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            direction = DcMotorSimple.Direction.REVERSE
        }
        rightBackDrive = hardwareMap.get(DcMotor::class.java, "right_back").apply {
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            direction = DcMotorSimple.Direction.REVERSE
        }

        imu = hardwareMap.get(IMU::class.java, "imu")

        shooter = ShooterPID(config.kP, config.kI, config.kD, config.kF)
        shooter.init(hardwareMap)


        ffl = hardwareMap.get(CRServo::class.java, "feed_left").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }

        ffr = hardwareMap.get(CRServo::class.java, "feed_right").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }

        intake = hardwareMap.get(DcMotorEx::class.java, "intake").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }

        val logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP
        val usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        val orientationOnRobot = RevHubOrientationOnRobot(logoDirection, usbDirection)

        imu.initialize(IMU.Parameters(orientationOnRobot))

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        val FORWARD_SPEED = 0.6

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run") //
        telemetry.update()

        // Wait for the game to start (driver presses START)
        waitForStart()
        runtime.reset()

        while (true) {
            shooter.setTargetRpm(2400.0)
            shooter.update()

            if (runtime.milliseconds() > 4) {
                // wait for the shooter to spin up
                ffr.power = -.75
                ffl.power = -.75
            }

            if (runtime.milliseconds() > 2) {
                // wait for the shooter to spin up
                intake.power = -.1

            }

            if (runtime.seconds() > 18) {
                intake.power = 0.0
                ffr.power = 0.0
                ffl.power = 0.0
                shooter.stop()
                break
            }
        }

        leftDrive.power = FORWARD_SPEED
        rightDrive.power = FORWARD_SPEED

        runtime.reset()
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds())
            telemetry.update()
        }

        // Step 4:  Stop
        leftDrive.power = 0.0
        rightDrive.power = 0.0

        telemetry.addData("Path", "Complete")
        telemetry.update()
        sleep(1000)
    }

}