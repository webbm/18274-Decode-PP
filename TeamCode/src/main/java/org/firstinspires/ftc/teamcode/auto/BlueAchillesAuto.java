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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.ShooterConfig;
import org.firstinspires.ftc.teamcode.robot.ShooterPID;

@Autonomous(name = "Far Blue Auto", group = "Blue")
@Configurable // Panels
public class BlueAchillesAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Timer pathTimer, actionTimer, opmodeTimer;

    private DcMotorEx intake;

    public ShooterConfig config;

    public ShooterPID shooterPID;

    private CRServo ffl;
    private CRServo ffr;

    public static class Paths {

        public PathChain readAprilTag;
        public PathChain scorePosition;
        public PathChain grabPickup1;
        public PathChain scorePickup1;
        public PathChain grabPickup2;
        public PathChain scorePickup2;
        public PathChain grabPickup3;
        public PathChain scorePickup3;
        public PathChain driveToGrab1;
        public PathChain park;

        //      Paths are in Here
        public Paths(Follower follower) {
//            readAprilTag = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(57.000, 9.000), new Pose(58.000, 103.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(80))
//                    .build();

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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

//    public void goToAprilTag() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(paths.readAprilTag);
//                setPathState(1);
//                break;
//            case 1:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
//                break;
//        }
//    }

    public void goToFirstScorePose() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.scorePosition, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()){
                   shooterPID.setTargetRpm(2300);
                   shooterPID.update();
                   actionTimer.resetTimer();
                   setPathState(2);
                }
                break;
            case 2:
                shooterPID.update();
                if (actionTimer.getElapsedTimeSeconds() > 0.75){
                    ffl.setPower(1);
                    ffr.setPower(1);
                    intake.setPower(0.15);
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                shooterPID.update();
                if(!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 7) {
                    shooterPID.setTargetRpm(0);
                    ffl.setPower(0);
                    ffr.setPower(0);
                    shooterPID.update();
                    follower.followPath(paths.grabPickup1, true);
                    setPathState(4);
                }
                break;
            case 4:
                shooterPID.update();
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    ffl.setPower(.3);
                    ffr.setPower(.3);
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (intake.getPower() > .9 && actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(paths.driveToGrab1, true);
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.5) {
                    intake.setPower(0.0);
                    ffl.setPower(0);
                    ffr.setPower(0);
                    follower.followPath(paths.scorePickup1, true);
                    setPathState(7);
                }
                break;
            case 7:
                shooterPID.update();
                if (!follower.isBusy()){
                    shooterPID.setTargetRpm(2200);
                    shooterPID.update();
                    actionTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                shooterPID.update();
                if (actionTimer.getElapsedTimeSeconds() > 1.5){
                    shooterPID.update();
                    intake.setPower(.15);
                    ffl.setPower(1);
                    ffr.setPower(1);
                    actionTimer.resetTimer();
                    shooterPID.update();
                    setPathState(9);
                }
                break;
            case 9:
                shooterPID.update();
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 7) {
                    shooterPID.setTargetRpm(0);
                    shooterPID.update();
                    ffl.setPower(0);
                    ffr.setPower(0);
                    shooterPID.update();
                    follower.followPath(paths.park, true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                shooterPID.update();
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    shooterPID.setTargetRpm(0.0);
                    shooterPID.update();
                    actionTimer.resetTimer();
                    if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(-1);
                    }
                }
                break;
        }
    }
    public void goToScore1Pose() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.scorePickup1);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }
    public void goToScore2Pose() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.scorePickup2);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }
    public void goToScore3Pose() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.scorePickup3);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    public void goToGrab1Pose() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.grabPickup1);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }
    public void goToGrab2Pose() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.grabPickup2);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }
    public void goToGrab3Pose() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.grabPickup3);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }
    public void Park() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.park);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
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

        shooterPID = new ShooterPID(config.kP, config.kI, config.kD, config.kF);
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
        goToFirstScorePose();

        //here we have to use the apriltag results to determine where
        //to go and what order to do it in using if and else if statements

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("intake power", intake.getPower());
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}