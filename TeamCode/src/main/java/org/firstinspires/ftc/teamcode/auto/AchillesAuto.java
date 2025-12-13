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
import com.sun.tools.javac.comp.Todo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.ToDoubleBiFunction;

@Autonomous(name = "Achilles Auto")
@Configurable // Panels
public class AchillesAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Timer pathTimer, actionTimer, opmodeTimer;

    public static class Paths {

        public PathChain readAprilTag;
        public PathChain scorePosition;
        public PathChain grabPickup1;
        public PathChain scorePickup1;
        public PathChain grabPickup2;
        public PathChain scorePickup2;
        public PathChain grabPickup3;
        public PathChain scorePickup3;
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
                            new BezierLine(new Pose(57.000, 103.000), new Pose(41.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();

            scorePickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.000, 84.000), new Pose(58.000, 103.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
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
                            new BezierLine(new Pose(58.000, 103.000), new Pose(36.000, 10.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void goToAprilTag() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.readAprilTag);
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

    public void goToFirstScorePose() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.scorePosition, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(paths.grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
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
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57, 15, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

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
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}