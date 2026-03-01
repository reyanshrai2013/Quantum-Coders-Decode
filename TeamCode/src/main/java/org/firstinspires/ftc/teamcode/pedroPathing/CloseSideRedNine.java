package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Importantthingsithasrizztrust.LauncherPIDF.coeffs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "CLOSE SIDE RED NINE", group = "Autonomous")
@Configurable
public class CloseSideRedNine extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private PathsForNineBallAuto paths;

    private long waitStartTime = 0;
    private long launcherStartTime = 0;
    private boolean waitStarted = false;
    private boolean pathStarted = false;

    private DcMotorEx launcher, launcher2, intake, feed;
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    private Limelight3A limelight;

    // Aiming constants
    private static final double ROTATION_KP = 0.05;
    private static final double TARGET_TOLERANCE = 1.5;
    private static final double MIN_ROTATION_POWER = 0.13;
    private static final double MAX_ROTATION_POWER = 0.4;
    private static final long AIM_TIMEOUT_MS = 2500;
    private static final int LOCK_CONFIRM_COUNT = 5;

    private boolean aimingStarted = false;
    private boolean aimDone = false;
    private long aimStartTime = 0;
    private int lockConfirmLoop = 0;

    @Override
    public void init() {

        launcher = hardwareMap.get(DcMotorEx.class, "launch");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launch1");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        feed = hardwareMap.get(DcMotorEx.class, "feed");

        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher2.setDirection(DcMotorEx.Direction.FORWARD);
        feed.setDirection(DcMotorEx.Direction.REVERSE);

        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
        launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(125.6945320197044, 122.38384729064042, Math.toRadians(36)));
        paths = new PathsForNineBallAuto(follower);
    }

    private void setLauncherVelocity(double v) {
        launcher.setVelocity(v);
        launcher2.setVelocity(v);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
    }

    private void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                if (!pathStarted) {
                    setLauncherVelocity(1090);
                    follower.followPath(paths.startToShoot, 0.6, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    waitStartTime = System.currentTimeMillis();
                    waitStarted = true;
                    pathStarted = false;
                    pathState = 1;
                }
                break;

            case 1:
                if (!pathStarted) {
                    follower.breakFollowing();
                    setLauncherVelocity(1090);
                    launcherStartTime = System.currentTimeMillis();
                    pathStarted = true;
                }

                if (System.currentTimeMillis() - launcherStartTime >= 500 && feed.getPower() == 0) {
                    feed.setPower(1);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait1) {
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStarted = false;
                    pathStarted = false;
                    pathState = 2;
                }
                break;

            case 2:
                if (!pathStarted) {
                    intake.setPower(1.0);
                    feed.setPower(-0.2);
                    setLauncherVelocity(10);
                    follower.followPath(paths.shootToFirstInkate, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    feed.setPower(-0.2);
                    setLauncherVelocity(1090);
                    pathStarted = false;
                    pathState = 4;
                }
                break;

            case 4:
                if (!pathStarted) {
                    intake.setPower(1);
                    follower.followPath(paths.firstIntakeToGate, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    pathStarted = false;
                    pathState = 5;
                }
                break;

            case 5:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    follower.followPath(paths.gateToShoot, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    pathStarted = false;
                    pathState = 6;
                }
                break;

            case 6:
                if (!pathStarted) {
                    setLauncherVelocity(1090);
                    launcherStartTime = System.currentTimeMillis();
                    waitStartTime = System.currentTimeMillis();
                    pathStarted = true;
                    waitStarted = true;
                }

                if (System.currentTimeMillis() - launcherStartTime >= 50 && feed.getPower() == 0) {
                    feed.setPower(0.75);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait1) {
                    setLauncherVelocity(1090);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStarted = false;
                    pathStarted = false;
                    pathState = 7;
                }
                break;

            case 7:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    follower.followPath(paths.Path7, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    pathStarted = false;
                    pathState = 8;
                }
                break;

            case 8:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    follower.followPath(paths.Path7ToSecondIntake, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    pathStarted = false;
                    pathState = 85;
                }
                break;

            case 85:
                if (!pathStarted) {
                    intake.setPower(0.5);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    follower.followPath(paths.SecondIntaketoGate, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    pathStarted = false;
                    pathState = 9;
                }
                break;

            case 9:
                if (!pathStarted) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    follower.followPath(paths.GateToShoot, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    pathStarted = false;
                    pathState = 10;
                }
                break;

            case 10:
                if (!pathStarted) {
                    setLauncherVelocity(1090);
                    launcherStartTime = System.currentTimeMillis();
                    waitStartTime = System.currentTimeMillis();
                    pathStarted = true;
                    waitStarted = true;
                }

                if (System.currentTimeMillis() - launcherStartTime >= 50 && feed.getPower() == 0) {
                    feed.setPower(0.75);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait1) {
                    setLauncherVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStarted = false;
                    pathStarted = false;
                    pathState = 11;
                }
                break;

            case 11:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    follower.followPath(paths.Path9, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    pathStarted = false;
                    pathState = 16;
                }
                break;

            case 16:
                break;
        }
    }

    // AIMING LOGIC
    private void runAimingLoop() {
        if (!aimingStarted || aimDone) return;

        if (System.currentTimeMillis() - aimStartTime >= AIM_TIMEOUT_MS) {
            mecanumDrive(0,0,0);
            aimDone = true;
            aimingStarted = false;
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            if (Math.abs(tx) < TARGET_TOLERANCE) {
                lockConfirmLoop++;
                mecanumDrive(0,0,0);
                if (lockConfirmLoop >= LOCK_CONFIRM_COUNT) {
                    aimDone = true;
                    aimingStarted = false;
                }
            } else {
                lockConfirmLoop = 0;
                double rot = tx * ROTATION_KP;
                if (Math.abs(rot) < MIN_ROTATION_POWER) {
                    rot = Math.signum(rot) * MIN_ROTATION_POWER;
                }
                rot = Math.max(-MAX_ROTATION_POWER, Math.min(MAX_ROTATION_POWER, rot));
                mecanumDrive(0,0,rot);
            }
        } else {
            mecanumDrive(0,0,0);
        }
    }

    private void startAim() {
        aimingStarted = true;
        aimDone = false;
        aimStartTime = System.currentTimeMillis();
        lockConfirmLoop = 0;
    }

    private void resetAim() {
        aimingStarted = false;
        aimDone = false;
        lockConfirmLoop = 0;
    }

    private boolean noTarget() {
        LLResult r = limelight.getLatestResult();
        return r == null || !r.isValid();
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        double d = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        leftFrontDrive.setPower((forward + strafe + rotate) / d);
        rightFrontDrive.setPower((forward - strafe - rotate) / d);
        leftBackDrive.setPower((forward - strafe + rotate) / d);
        rightBackDrive.setPower((forward + strafe - rotate) / d);
    }

    // KEEP YOUR ORIGINAL PATHS CLASS BELOW (UNCHANGED)
}

