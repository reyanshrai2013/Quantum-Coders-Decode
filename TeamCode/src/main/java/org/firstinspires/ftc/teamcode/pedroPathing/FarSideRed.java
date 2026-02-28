package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Autonomous(name = "RED FAR SIDE", group = "Autonomous")
public class FarSideRed extends OpMode {

    // ── Pedro Pathing ─────────────────────────────────────────────────────────
    private Follower follower;
    private Paths    paths;
    private int      pathState   = 0;
    private boolean  pathStarted = false;

    // ── Timing ────────────────────────────────────────────────────────────────
    private long    waitStartTime     = 0;
    private long    launcherStartTime = 0;
    private boolean waitStarted       = false;

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

    // Aiming constants
    private static final double ROTATION_KP        = 0.05;
    private static final double TARGET_TOLERANCE   = 1.5;  // tightened: was 2.0
    private static final double MIN_ROTATION_POWER = 0.13; // lowered: less overshoot
    private static final double MAX_ROTATION_POWER = 0.4;  // lowered: more accurate
    private static final long   AIM_TIMEOUT_MS     = 2500; // slightly more time to settle
    // Must stay on-target this many consecutive loops before declaring locked
    private static final int    LOCK_CONFIRM_COUNT = 5;

    // Aiming state
    private boolean aimingStarted   = false;
    private boolean aimDone         = false;
    private long    aimStartTime    = 0;
    private int     lockConfirmLoop = 0;

    // ─────────────────────────────────────────────────────────────────────────
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

        launcherRight.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(200, 0, 0, 12.1));
        launcherLeft.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(200, 0, 0, 12.1));
        // ── Intake + feed ─────────────────────────────────────────────────────
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        feed   = hardwareMap.get(DcMotorEx.class, "feed");

        feed.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(BRAKE);
        feed.setZeroPowerBehavior(BRAKE);

        // ── Limelight — exactly as in working TeleOp ──────────────────────────
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X",       follower.getPose().getX());
        telemetry.addData("Y",       follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        LLResult dbg = limelight.getLatestResult();
        if (dbg != null && dbg.isValid()) {
            telemetry.addData("LL Target", String.format("YES  tx=%.2f deg", dbg.getTx()));
        } else {
            telemetry.addData("LL Target", "none");
        }
        telemetry.addData("Aim",       aimDone ? "DONE" : aimingStarted ? "aiming" : "idle");
        telemetry.addData("LockCount", lockConfirmLoop);
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    private void autonomousPathUpdate() {

        switch (pathState) {

            // ── 0: drive to first shooting position ───────────────────────────
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

            // ── 1: first shot ─────────────────────────────────────────────────
            case 1:
                if (!pathStarted) {
                    follower.breakFollowing(); // stop Pedro fighting the aiming rotation
                    setLauncherVelocity(1460);
                    launcherStartTime = System.currentTimeMillis();
                    startAim();
                    pathStarted = true;
                }

                runAimingLoop();

                if (System.currentTimeMillis() - launcherStartTime >= 1600
                        && (aimDone || noTarget())) {
                    feed.setPower(1);
                    intake.setPower(1.0);
                }

                if (waitStarted
                        && System.currentTimeMillis() - waitStartTime >= paths.Wait3) {
                    setLauncherVelocity(1460);
                    feed.setPower(0);
                    intake.setPower(0);
                    resetAim();
                    waitStarted = false;
                    pathStarted = false;
                    pathState   = 2;
                }
                break;

            // ── 2: move to ball stack ─────────────────────────────────────────
            case 2:
                if (!pathStarted) {
                    follower.followPath(paths.Path3, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState   = 3;
                }
                break;

            // ── 3: intake first ball ──────────────────────────────────────────
            case 3:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1500);
                    follower.followPath(paths.Path4, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1500);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted   = true;
                    pathStarted   = false;
                    pathState     = 4;
                }
                break;

            // ── 4: return to shoot ────────────────────────────────────────────
            case 4:
                if (!pathStarted) {
                    follower.followPath(paths.Path5, true);
                    intake.setPower(0);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    pathStarted = false;
                    pathState   = 5;
                }
                break;

            // ── 5: second shot ────────────────────────────────────────────────
            case 5:
                if (!pathStarted) {
                    follower.breakFollowing(); // stop Pedro fighting the aiming rotation
                    setLauncherVelocity(1460);
                    launcherStartTime = System.currentTimeMillis();
                    waitStartTime     = System.currentTimeMillis();
                    startAim();
                    waitStarted = true;
                    pathStarted = true;
                }

                runAimingLoop();

                if (System.currentTimeMillis() - launcherStartTime >=750
                        && feed.getPower() == 0
                        && (aimDone || noTarget())) {
                    feed.setPower(1);
                    intake.setPower(1.0);
                }

                if (waitStarted
                        && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {
                    setLauncherVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    resetAim();
                    waitStarted = false;
                    pathStarted = false;
                    pathState   = 6;
                }
                break;

            // ── 6: swing toward second ball stack ─────────────────────────────
            case 6:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1450);
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

            // ── 7: sweep second ball stack ────────────────────────────────────
            case 7:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1450);
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

            // ── 8: return to shoot ────────────────────────────────────────────
            case 8:
                if (!pathStarted) {
                    intake.setPower(1.0);
                    feed.setPower(0);
                    setLauncherVelocity(1500);
                    follower.followPath(paths.Path9, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1500);
                    waitStartTime = System.currentTimeMillis();
                    waitStarted   = true;
                    pathStarted   = false;
                    pathState     = 9;
                }
                break;

            // ── 9: third shot ─────────────────────────────────────────────────
            case 9:
                if (!pathStarted) {
                    follower.breakFollowing(); // stop Pedro fighting the aiming rotation
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
                    feed.setPower(1);
                    intake.setPower(1.0);
                }

                if (waitStarted
                        && System.currentTimeMillis() - waitStartTime >= paths.Wait2) {
                    setLauncherVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    resetAim();
                    waitStarted = false;
                    pathStarted = false;
                    pathState   = 10;
                }
                break;

            // ── 10: park ──────────────────────────────────────────────────────
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
    // AIMING — direct port of autoTrackAprilTag() from working TeleOp
    //          + confirmation counter to prevent locking on a single noisy frame
    // =========================================================================
    private void runAimingLoop() {
        if (!aimingStarted || aimDone) return;

        // Hard timeout — give up and fire anyway
        if (System.currentTimeMillis() - aimStartTime >= AIM_TIMEOUT_MS) {
            mecanumDrive(0, 0, 0);
            aimDone       = true;
            aimingStarted = false;
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // horizontal offset in degrees

            if (Math.abs(tx) < TARGET_TOLERANCE) {
                // Must stay on-target LOCK_CONFIRM_COUNT loops in a row before locking.
                // Prevents firing immediately on a single noisy reading.
                lockConfirmLoop++;
                mecanumDrive(0, 0, 0); // hold still while confirming
                if (lockConfirmLoop >= LOCK_CONFIRM_COUNT) {
                    aimDone       = true;
                    aimingStarted = false;
                }
            } else {
                // Off target — reset confirmation counter and keep rotating
                lockConfirmLoop = 0;

                double rot = tx * ROTATION_KP;
                if (Math.abs(rot) < MIN_ROTATION_POWER) {
                    rot = Math.signum(rot) * MIN_ROTATION_POWER;
                }
                rot = Math.max(-MAX_ROTATION_POWER, Math.min(MAX_ROTATION_POWER, rot));

                mecanumDrive(0, 0, rot);
            }
        } else {
            // No target — hold still (robot is near target from path ending;
            // spinning to search was causing the progressive drift between shots)
            lockConfirmLoop = 0;
            mecanumDrive(0, 0, 0);
        }
    }

    // ── Aiming helpers ────────────────────────────────────────────────────────
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

    // ── Paths ─────────────────────────────────────────────────────────────────
    public static class Paths {

        public PathChain Path1, Path3, Path4, Path5, Path7, Path8, Path9, Path11;
        public double Wait2 = 1750;
        public double Wait3 = 3750;


        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 56.879, 8.412),
                            new Pose(144 - 54.6828929, 16.22253129)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(61))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 54.6828929, 16.22253129),
                            new Pose(144 - 49.0681502, 35.4492350)))
                    .setLinearHeadingInterpolation(Math.toRadians(61), Math.toRadians(-3))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 49.0681502, 35.4492350),
                            new Pose(144 - 8.11307371349096, 35.64951321)))
                    .setLinearHeadingInterpolation(Math.toRadians(-3), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 8.11307371349096, 35.64951321),
                            new Pose(144 - 58.6828929, 21.22253129)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 52.6828929, 20.22253129),
                            new Pose(144 - 41.458, 8.217)))
                    .setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(0))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 41.458, 8.217),
                            new Pose(144 - 10.214, 6.217)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 16.214, 6.217),
                            new Pose(144 - 59, 16.773)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(83))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(144 - 59, 16.773),
                            new Pose(84.551724137931, 37.06502463054187)))
                    .setLinearHeadingInterpolation(Math.toRadians(88), Math.toRadians(-20))
                    .build();
        }
    }
}