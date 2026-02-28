package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Importantthingsithasrizztrust.LauncherPIDF.coeffs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous(name = "FAR SIDE RED NO SPIKE", group = "No Spike")
@Configurable
public class FarSideRedNoSpike extends OpMode {

    private TelemetryManager panelsTelemetry;

    // ── Pedro Pathing ─────────────────────────────────────────────────────────
    public Follower follower;
    private Paths paths;
    private int pathState = 0;
    private boolean pathStarted = false;

    // ── Timing ────────────────────────────────────────────────────────────────
    private long waitStartTime     = 0;
    private long launcherStartTime = 0;
    private boolean waitStarted    = false;

    // ── Motors ────────────────────────────────────────────────────────────────
    private DcMotorEx launcherRight = null;
    private DcMotorEx launcherLeft  = null;
    private DcMotorEx intake        = null;
    private DcMotorEx feed          = null;

    private DcMotor leftFrontDrive  = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightBackDrive  = null;

    // ── Limelight ─────────────────────────────────────────────────────────────
    private Limelight3A limelight = null;

    // Aiming constants (mirrored from FarSideRed)
    private static final double ROTATION_KP        = 0.05;
    private static final double TARGET_TOLERANCE   = 1.5;
    private static final double MIN_ROTATION_POWER = 0.13;
    private static final double MAX_ROTATION_POWER = 0.4;
    private static final long   AIM_TIMEOUT_MS     = 2500;
    private static final int    LOCK_CONFIRM_COUNT = 5;

    // Aiming state
    private boolean aimingStarted   = false;
    private boolean aimDone         = false;
    private long    aimStartTime    = 0;
    private int     lockConfirmLoop = 0;


    @Override
    public void init() {

        // ── Pedro Pathing follower ────────────────────────────────────────────
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144 - 56.879, 8.412, Math.toRadians(90)));
        paths = new Paths(follower);

        // ── Drive motors ──────────────────────────────────────────────────────
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rb");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        // ── Launchers ─────────────────────────────────────────────────────────
        launcherRight = hardwareMap.get(DcMotorEx.class, "launch");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launch1");

        launcherRight.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setDirection(DcMotor.Direction.FORWARD);

        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherRight.setZeroPowerBehavior(BRAKE);
        launcherLeft.setZeroPowerBehavior(BRAKE);

        launcherRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
        launcherLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);

        // ── Intake + feed ─────────────────────────────────────────────────────
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        feed   = hardwareMap.get(DcMotorEx.class, "feed");

        feed.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(BRAKE);
        feed.setZeroPowerBehavior(BRAKE);

        // ── Limelight ─────────────────────────────────────────────────────────
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }


    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X",       follower.getPose().getX());
        panelsTelemetry.debug("Y",       follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));

        LLResult dbg = limelight.getLatestResult();
        if (dbg != null && dbg.isValid()) {
            panelsTelemetry.debug("LL Target", String.format("YES  tx=%.2f deg", dbg.getTx()));
        } else {
            panelsTelemetry.debug("LL Target", "none");
        }
        panelsTelemetry.debug("Aim",       aimDone ? "DONE" : aimingStarted ? "aiming" : "idle");
        panelsTelemetry.debug("LockCount", lockConfirmLoop);
        panelsTelemetry.update(telemetry);
    }


    private void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                if (!pathStarted) {
                    follower.followPath(paths.Path1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    waitStartTime = System.currentTimeMillis();
                    waitStarted   = true;
                    pathStarted   = false;
                    pathState     = 1;
                }
                break;

            case 1:
                if (!pathStarted) {
                    follower.breakFollowing();
                    setLauncherVelocity(1460);
                    launcherStartTime = System.currentTimeMillis();
                    startAim();
                    pathStarted = true;
                }

                runAimingLoop();

                if (System.currentTimeMillis() - launcherStartTime >= 1600
                        && (aimDone || noTarget())) {
                    feed.setPower(1.0);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {
                    setLauncherVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    resetAim();
                    waitStarted = false;
                    pathStarted = false;
                    pathState   = 2;
                }
                break;

            case 2:
                if (!pathStarted) {
                    intake.setPower(0.8);
                    feed.setPower(0.25);
                    setLauncherVelocity(-700);
                    follower.followPath(paths.Path3, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(0);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted   = true;
                    pathStarted   = false;
                    pathState     = 3;
                }
                break;

            case 3:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0.25);
                    setLauncherVelocity(-700);
                    follower.followPath(paths.Path4, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(0);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted   = true;
                    pathStarted   = false;
                    pathState     = 4;
                }
                break;

            case 4:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0.25);
                    setLauncherVelocity(-700);
                    follower.followPath(paths.Path5, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(0);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted   = true;
                    pathStarted   = false;
                    pathState     = 5;
                }
                break;

            case 5:
                if (!pathStarted) {
                    follower.breakFollowing();
                    setLauncherVelocity(1460);
                    launcherStartTime = System.currentTimeMillis();
                    waitStartTime     = System.currentTimeMillis();
                    startAim();
                    waitStarted = true;
                    pathStarted = true;
                }

                runAimingLoop();

                if (System.currentTimeMillis() - launcherStartTime >= 750
                        && feed.getPower() == 0
                        && (aimDone || noTarget())) {
                    feed.setPower(1.0);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {
                    setLauncherVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    resetAim();
                    waitStarted = false;
                    pathStarted = false;
                    pathState   = 6;
                }
                break;

            case 6:
                if (!pathStarted) {
                    intake.setPower(0.8);
                    feed.setPower(0.25);
                    setLauncherVelocity(-700);
                    follower.followPath(paths.Path7, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(0);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted   = true;
                    pathStarted   = false;
                    pathState     = 7;
                }
                break;

            case 7:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0.25);
                    setLauncherVelocity(-700);
                    follower.followPath(paths.Path8, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(0);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted   = true;
                    pathStarted   = false;
                    pathState     = 8;
                }
                break;

            case 8:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0.25);
                    setLauncherVelocity(-700);
                    follower.followPath(paths.Path9, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(0);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted   = true;
                    pathStarted   = false;
                    pathState     = 9;
                }
                break;

            case 9:
                if (!pathStarted) {
                    follower.breakFollowing();
                    setLauncherVelocity(1500);
                    launcherStartTime = System.currentTimeMillis();
                    waitStartTime     = System.currentTimeMillis();
                    startAim();
                    waitStarted = true;
                    pathStarted = true;
                }

                runAimingLoop();

                if (System.currentTimeMillis() - launcherStartTime >= 1600
                        && feed.getPower() == 0
                        && (aimDone || noTarget())) {
                    feed.setPower(1.0);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {
                    setLauncherVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    resetAim();
                    waitStarted = false;
                    pathStarted = false;
                    pathState   = 10;
                }
                break;

            case 10:
                if (!pathStarted) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(0);
                    follower.followPath(paths.Path11, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setLauncherVelocity(0);
                    pathStarted = false;
                    pathState   = 11;
                }
                break;
        }
    }

    // =========================================================================
    // AIMING — direct port from FarSideRed with lock confirmation counter
    // =========================================================================
    private void runAimingLoop() {
        if (!aimingStarted || aimDone) return;

        if (System.currentTimeMillis() - aimStartTime >= AIM_TIMEOUT_MS) {
            mecanumDrive(0, 0, 0);
            aimDone       = true;
            aimingStarted = false;
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            if (Math.abs(tx) < TARGET_TOLERANCE) {
                lockConfirmLoop++;
                mecanumDrive(0, 0, 0);
                if (lockConfirmLoop >= LOCK_CONFIRM_COUNT) {
                    aimDone       = true;
                    aimingStarted = false;
                }
            } else {
                lockConfirmLoop = 0;
                double rot = tx * ROTATION_KP;
                if (Math.abs(rot) < MIN_ROTATION_POWER) {
                    rot = Math.signum(rot) * MIN_ROTATION_POWER;
                }
                rot = Math.max(-MAX_ROTATION_POWER, Math.min(MAX_ROTATION_POWER, rot));
                mecanumDrive(0, 0, rot);
            }
        } else {
            // No target — hold still, don't spin and drift
            lockConfirmLoop = 0;
            mecanumDrive(0, 0, 0);
        }
    }

    private void startAim() {
        aimingStarted   = true;
        aimDone         = false;
        aimStartTime    = System.currentTimeMillis();
        lockConfirmLoop = 0;
    }

    private void resetAim() {
        aimingStarted   = false;
        aimDone         = false;
        lockConfirmLoop = 0;
    }

    private boolean noTarget() {
        LLResult r = limelight.getLatestResult();
        return r == null || !r.isValid();
    }

    // ── Motor helpers ─────────────────────────────────────────────────────────
    private void setLauncherVelocity(double velocity) {
        launcherRight.setVelocity(velocity);
        launcherLeft.setVelocity(velocity);
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        double d = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        leftFrontDrive.setPower( (forward + strafe + rotate) / d);
        rightFrontDrive.setPower((forward - strafe - rotate) / d);
        leftBackDrive.setPower(  (forward - strafe + rotate) / d);
        rightBackDrive.setPower( (forward + strafe - rotate) / d);
    }

    // ── Paths (unchanged from original NoSpike) ───────────────────────────────
    public static class Paths {

        public PathChain Path1, Path3, Path4, Path5, Path7, Path8, Path9, Path11;
        public double Wait2 = 3750;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 56.879, 8.412),
                            new Pose(144 - 59.6828929, 16.22253129)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(61))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 58.481, 18.826),
                            new Pose(144 - 41.458, 12.217)))
                    .setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(-20))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 41.458, 12.217),
                            new Pose(144 - 10.214, 12.217)))
                    .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(-20))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 10.214, 12.217),
                            new Pose(144 - 59, 19.773)))
                    .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(83))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 58.481, 18.826),
                            new Pose(144 - 41.458, 12.217)))
                    .setLinearHeadingInterpolation(Math.toRadians(83), Math.toRadians(-20))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 41.458, 12.217),
                            new Pose(144 - 10.214, 12.217)))
                    .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(-20))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 10.214, 12.217),
                            new Pose(144 - 59, 19.773)))
                    .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(83))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 59, 19.773),
                            new Pose(84.551724137931, 37.06502463054187)))
                    .setLinearHeadingInterpolation(Math.toRadians(88), Math.toRadians(-20))
                    .build();
        }
    }
}