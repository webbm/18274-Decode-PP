package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.ShooterConfig;
import org.firstinspires.ftc.teamcode.robot.ShooterPID;

@Autonomous(name = "Far Blue High Score Auto", group = "Blue")
@Configurable // Panels
@Disabled
public class BlueAchillesHighScoreAuto extends OpMode {

    public static double PRELOAD_RPM = 2300;
    public static double CYCLE_RPM = 2200;
    public static double SPINUP_TIME_SEC = 0.75;
    public static double FEED_TIME_SEC = 1.5;
    public static double INTAKE_TIME_SEC = 2.0;
    public static double DRIVE_TO_GRAB_SETTLE_SEC = 0.5;
    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_FEED_POWER = 0.3;
    public static double FEED_POWER = 1.0;
    public static double INTAKE_SHOOT_POWER = 0.15;

    protected TelemetryManager panelsTelemetry; // Panels Telemetry instance
    protected Follower follower; // Pedro Pathing follower instance
    protected int pathState; // Current autonomous path state (state machine)
    protected Paths paths; // Paths defined in the Paths class

    protected Timer pathTimer, actionTimer, opmodeTimer;

    protected DcMotorEx intake;
    protected ShooterPID shooterPID;
    protected CRServo ffl;
    protected CRServo ffr;

    public static class Paths {
        public PathChain scorePosition;
        public PathChain grabPickup1;
        public PathChain scorePickup1;
        public PathChain grabPickup2;
        public PathChain scorePickup2;
        public PathChain grabPickup3;
        public PathChain scorePickup3;
        public PathChain driveToGrab1;
        public PathChain park;

        public Paths(Follower follower) {
            driveToGrab1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 86.500), new Pose(27.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            scorePosition = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.000, 9.000), new Pose(57.000, 103.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
                    .build();

            grabPickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.000, 103.000), new Pose(48.000, 90.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0))
                    .build();

            scorePickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(30.000, 86.500), new Pose(58.000, 103.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(140))
                    .build();

            grabPickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 103.000), new Pose(41.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();

            scorePickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.000, 60.000), new Pose(58.000, 103.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();

            grabPickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 103.000), new Pose(41.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();

            scorePickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.000, 36.000), new Pose(58.000, 103.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();

            park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 103.000), new Pose(60.000, 140.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();
        }
    }

    protected void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    protected void setFeedPower(double power) {
        ffl.setPower(power);
        ffr.setPower(power);
    }

    protected void startShooting(double rpm) {
        shooterPID.setTargetRpm(rpm);
        shooterPID.update();
        actionTimer.resetTimer();
    }

    protected void startFeeding() {
        setFeedPower(FEED_POWER);
        intake.setPower(INTAKE_SHOOT_POWER);
        actionTimer.resetTimer();
    }

    protected void stopFeeding() {
        setFeedPower(0);
        intake.setPower(0.0);
    }

    protected void startIntake() {
        intake.setPower(INTAKE_POWER);
        setFeedPower(INTAKE_FEED_POWER);
        actionTimer.resetTimer();
    }

    protected void stopIntake() {
        intake.setPower(0.0);
        setFeedPower(0);
    }

    protected boolean readyToShoot() {
        return true;
    }

    protected void runHighScoreAuto() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.scorePosition, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && readyToShoot()) {
                    startShooting(PRELOAD_RPM);
                    setPathState(2);
                }
                break;
            case 2:
                if (actionTimer.getElapsedTimeSeconds() > SPINUP_TIME_SEC) {
                    startFeeding();
                    setPathState(3);
                }
                break;
            case 3:
                if (actionTimer.getElapsedTimeSeconds() > FEED_TIME_SEC) {
                    stopFeeding();
                    shooterPID.setTargetRpm(0);
                    follower.followPath(paths.grabPickup1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    startIntake();
                    setPathState(5);
                }
                break;
            case 5:
                if (actionTimer.getElapsedTimeSeconds() > INTAKE_TIME_SEC) {
                    follower.followPath(paths.driveToGrab1, true);
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > DRIVE_TO_GRAB_SETTLE_SEC) {
                    stopIntake();
                    follower.followPath(paths.scorePickup1, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && readyToShoot()) {
                    startShooting(CYCLE_RPM);
                    setPathState(8);
                }
                break;
            case 8:
                if (actionTimer.getElapsedTimeSeconds() > SPINUP_TIME_SEC) {
                    startFeeding();
                    setPathState(9);
                }
                break;
            case 9:
                if (actionTimer.getElapsedTimeSeconds() > FEED_TIME_SEC) {
                    stopFeeding();
                    shooterPID.setTargetRpm(0);
                    follower.followPath(paths.grabPickup2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    startIntake();
                    setPathState(11);
                }
                break;
            case 11:
                if (actionTimer.getElapsedTimeSeconds() > INTAKE_TIME_SEC) {
                    stopIntake();
                    follower.followPath(paths.scorePickup2, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() && readyToShoot()) {
                    startShooting(CYCLE_RPM);
                    setPathState(13);
                }
                break;
            case 13:
                if (actionTimer.getElapsedTimeSeconds() > SPINUP_TIME_SEC) {
                    startFeeding();
                    setPathState(14);
                }
                break;
            case 14:
                if (actionTimer.getElapsedTimeSeconds() > FEED_TIME_SEC) {
                    stopFeeding();
                    shooterPID.setTargetRpm(0);
                    follower.followPath(paths.grabPickup3, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    startIntake();
                    setPathState(16);
                }
                break;
            case 16:
                if (actionTimer.getElapsedTimeSeconds() > INTAKE_TIME_SEC) {
                    stopIntake();
                    follower.followPath(paths.scorePickup3, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy() && readyToShoot()) {
                    startShooting(CYCLE_RPM);
                    setPathState(18);
                }
                break;
            case 18:
                if (actionTimer.getElapsedTimeSeconds() > SPINUP_TIME_SEC) {
                    startFeeding();
                    setPathState(19);
                }
                break;
            case 19:
                if (actionTimer.getElapsedTimeSeconds() > FEED_TIME_SEC) {
                    stopFeeding();
                    shooterPID.setTargetRpm(0);
                    follower.followPath(paths.park, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    shooterPID.setTargetRpm(0);
                    shooterPID.update();
                    setPathState(-1);
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        actionTimer = new Timer();
        actionTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57, 15, Math.toRadians(90)));

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        paths = new Paths(follower); // Build paths

        shooterPID = new ShooterPID(ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD, ShooterConfig.kF);
        shooterPID.init(hardwareMap);

        ffl = hardwareMap.get(CRServo.class, "feed_left");
        ffr = hardwareMap.get(CRServo.class, "feed_right");

        ffl.setDirection(DcMotorSimple.Direction.REVERSE);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        shooterPID.update();
        runHighScoreAuto();

        panelsTelemetry.debug("intake power", intake.getPower());
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}
