package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.util.Timer;

import java.util.List;

@Autonomous(name = "Far Blue High Score AprilTag Auto", group = "Blue")
public class BlueAchillesHighScoreAprilTagAuto extends BlueAchillesHighScoreAuto {

    public static int LIMELIGHT_PIPELINE = 0;
    public static int TARGET_TAG_ID = 20;
    public static double TARGET_TX_DEG = 0.0;
    public static double TARGET_TY_DEG = 0.0;
    public static double TX_TOLERANCE_DEG = 1.5;
    public static double TY_TOLERANCE_DEG = 1.5;
    public static double ALIGN_TURN_KP = 0.02;
    public static double ALIGN_DRIVE_KP = 0.02;
    public static double MAX_TURN_POWER = 0.25;
    public static double MAX_DRIVE_POWER = 0.25;
    public static double ALIGN_TIMEOUT_SEC = 1.5;

    private Limelight3A limelight;
    private Timer alignTimer;
    private int lastAlignPathState = -1;

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;

    @Override
    public void init() {
        super.init();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        alignTimer = new Timer();
        alignTimer.resetTimer();

        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        super.start();
        limelight.start();
    }

    @Override
    public void stop() {
        super.stop();
        limelight.stop();
        setDrivePower(0.0, 0.0, 0.0);
    }

    @Override
    protected boolean readyToShoot() {
        if (pathState != lastAlignPathState) {
            lastAlignPathState = pathState;
            alignTimer.resetTimer();
        }

        if (alignTimer.getElapsedTimeSeconds() > ALIGN_TIMEOUT_SEC) {
            setDrivePower(0.0, 0.0, 0.0);
            return true;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            setDrivePower(0.0, 0.0, 0.0);
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            setDrivePower(0.0, 0.0, 0.0);
            return false;
        }

        LLResultTypes.FiducialResult targetFiducial = fiducials.get(0);
        if (TARGET_TAG_ID >= 0) {
            boolean foundTarget = false;
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                    targetFiducial = fiducial;
                    foundTarget = true;
                    break;
                }
            }
            if (!foundTarget) {
                setDrivePower(0.0, 0.0, 0.0);
                return false;
            }
        }

        double txError = TARGET_TX_DEG - targetFiducial.getTargetXDegrees();
        double tyError = TARGET_TY_DEG - targetFiducial.getTargetYDegrees();

        boolean txAligned = Math.abs(txError) <= TX_TOLERANCE_DEG;
        boolean tyAligned = Math.abs(tyError) <= TY_TOLERANCE_DEG;

        if (txAligned && tyAligned) {
            setDrivePower(0.0, 0.0, 0.0);
            return true;
        }

        double turn = Range.clip(txError * ALIGN_TURN_KP, -MAX_TURN_POWER, MAX_TURN_POWER);
        double drive = Range.clip(tyError * ALIGN_DRIVE_KP, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        setDrivePower(drive, 0.0, turn);
        return false;
    }

    private void setDrivePower(double forward, double strafe, double turn) {
        double leftFrontPower = forward + strafe + turn;
        double rightFrontPower = forward - strafe - turn;
        double leftBackPower = forward - strafe + turn;
        double rightBackPower = forward + strafe - turn;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
}
