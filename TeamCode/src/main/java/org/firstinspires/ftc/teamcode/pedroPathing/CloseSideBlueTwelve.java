package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Importantthingsithasrizztrust.LauncherPIDF.coeffs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name = "CLOSE SIDE BLUE TWELVE", group = "Autonomous")
@Configurable

public class CloseSideBlueTwelve extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    private long waitStartTime = 0;
    private long launcherStartTime = 0;
    private boolean waitStarted = false;
    private boolean pathStarted = false;

    private DcMotorEx launcher = null;
    private DcMotorEx launcher2 = null;
    private DcMotorEx intake = null;
    private DcMotorEx feed = null;

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

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Red: (125.69, 122.38, 36°)  →  Blue: (125.69, 144-122.38=21.62, -36°)
        follower.setStartingPose(new Pose(125.6945320197044, 21.616152709359578, Math.toRadians(-36)));
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    private void setLauncherVelocity(double v) {
        launcher.setVelocity(v);
        launcher2.setVelocity(v);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    // autonomousPathUpdate() is identical in structure — no changes needed there.
    // All motor logic, state machine, and timing are alliance-agnostic.
    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (!pathStarted) {
                    setLauncherVelocity(1070);
                    follower.followPath(paths.startToShoot, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    waitStartTime = System.currentTimeMillis();
                    waitStarted = true;
                    pathStarted = false;
                    pathState = 1;
                }
                break;

            case 1:
                if (!pathStarted) {
                    setLauncherVelocity(1125);
                    launcherStartTime = System.currentTimeMillis();
                    pathStarted = true;
                }

                if (System.currentTimeMillis() - launcherStartTime >= 500 && feed.getPower() == 0) {
                    feed.setPower(1);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait1) {
                    setLauncherVelocity(1070);
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
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(1);
                    feed.setPower(-0.2);
                    setLauncherVelocity(1070);
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
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(1);
                    pathStarted = false;
                    pathState = 5;
                }
                break;

            case 5:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1070);
                    follower.followPath(paths.gateToShoot, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1070);
                    pathStarted = false;
                    pathState = 6;
                }
                break;

            case 6:
                if (!pathStarted) {
                    setLauncherVelocity(1070);
                    launcherStartTime = System.currentTimeMillis();
                    waitStartTime = System.currentTimeMillis();
                    pathStarted = true;
                    waitStarted = true;
                }

                if (System.currentTimeMillis() - launcherStartTime >= 50 && feed.getPower() == 0) {
                    feed.setPower(1);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait1) {
                    setLauncherVelocity(1070);
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
                    setLauncherVelocity(1070);
                    follower.followPath(paths.Path7, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1070);
                    pathStarted = false;
                    pathState = 8;
                }
                break;

            case 8:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1070);
                    follower.followPath(paths.Path7ToSecondIntake, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1070);
                    pathStarted = false;
                    pathState = 85;
                }
                break;

            case 85:
                if (!pathStarted) {
                    intake.setPower(0.5);
                    feed.setPower(0);
                    setLauncherVelocity(1070);
                    follower.followPath(paths.SecondIntaketoGate, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1070);
                    pathStarted = false;
                    pathState = 9;
                }
                break;

            case 9:
                if (!pathStarted) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1070);
                    follower.followPath(paths.GateToShoot, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1070);
                    pathStarted = false;
                    pathState = 10;
                }
                break;

            case 10:
                if (!pathStarted) {
                    setLauncherVelocity(1070);
                    launcherStartTime = System.currentTimeMillis();
                    waitStartTime = System.currentTimeMillis();
                    pathStarted = true;
                    waitStarted = true;
                }

                if (System.currentTimeMillis() - launcherStartTime >= 50 && feed.getPower() == 0) {
                    feed.setPower(1);
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
                    intake.setPower(1.0);
                    feed.setPower(0);
                    setLauncherVelocity(1100);
                    follower.followPath(paths.shootToThirdIntake, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1150);
                    pathStarted = false;
                    pathState = 13;
                }
                break;

            case 13:
                if (!pathStarted) {
                    setLauncherVelocity(1100);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStartTime = System.currentTimeMillis();
                    pathStarted = true;
                    waitStarted = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    setLauncherVelocity(1100);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStarted = false;
                    pathStarted = false;
                    pathState = 14;
                }
                break;

            case 15:
                if (!pathStarted) {
                    setLauncherVelocity(1100);
                    launcherStartTime = System.currentTimeMillis();
                    waitStartTime = System.currentTimeMillis();
                    pathStarted = true;
                    waitStarted = true;
                }

                if (System.currentTimeMillis() - launcherStartTime >= 50 && feed.getPower() == 0) {
                    feed.setPower(1);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait1) {
                    setLauncherVelocity(0);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStarted = false;
                    pathStarted = false;
                    pathState = 16;
                }
                break;

            case 14:
                if (!pathStarted) {
                    follower.followPath(paths.Path9, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    pathStarted = false;
                    pathState = 15;
                }
                break;

            case 16:
                break;
        }
    }

    public static class Paths {
        public PathChain startToShoot;
        public PathChain shootToFirstInkate;
        public PathChain firstIntakeToGate;
        public PathChain gateToShoot;
        public PathChain Path7ToSecondIntake;
        public PathChain GateToShoot;
        public PathChain shootToThirdIntake;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain SecondIntaketoGate;

        public double Wait1;
        public double Wait2;

        public Paths(Follower follower) {

            Wait1 = 1750;

            // ---------------------------------------------------------------
            // MIRROR RULE: Y_blue = 144 - Y_red,  heading_blue = -heading_red
            // ---------------------------------------------------------------

            // Red start: (125.69, 122.38, 36°)  →  Blue: (125.69, 21.62, -36°)
            // Red end:   (96.84,  96.77, 45°)   →  Blue: (96.84,  47.23, -45°)
            startToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(125.6945320197044, 21.616152709359578),
                                    new Pose(96.84280662983427, 47.22966850828729)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-45))
                    .setVelocityConstraint(25)
                    .build();

            // Red: (85.97, 82.83, 0°) → (132.19, 82.84, 0°)
            // Blue: (85.97, 61.17, 0°) → (132.19, 61.16, 0°)
            shootToFirstInkate = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.970, 61.167),
                                    new Pose(132.19211822660097, 61.15763546798027)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Red control points: (132.19, 84.84), (108.685, 75.453), (132.547, 75.621)
            // Blue: (132.19, 59.16), (108.685, 68.547), (132.547, 68.379)
            firstIntakeToGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(132.19211822660097, 59.15763546798027),
                                    new Pose(108.685, 68.547),
                                    new Pose(132.547, 68.379)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Red: (128.547, 72.621, 0°) → (96.8, 96.77, 34°)
            // Blue: (128.547, 71.379, 0°) → (96.8, 47.23, -34°)
            gateToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.547, 71.379),
                                    new Pose(96.8, 47.23)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-34))
                    .build();

            // Red: (85.675, 85.611, 38°) → (100.419, 58.596, 0°)
            // Blue: (85.675, 58.389, -38°) → (100.419, 85.404, 0°)
            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.675, 58.389),
                                    new Pose(100.419, 85.404)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-38), Math.toRadians(0))
                    .build();

            // Red: (100.419, 59.596, 0°) → (141.990, 58.488, 0°)
            // Blue: (100.419, 84.404, 0°) → (141.990, 85.512, 0°)
            Path7ToSecondIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.419, 84.404),
                                    new Pose(141.99014778325125, 85.51231527093597)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Red control points: (141.990, 58.488), (94.577, 55.557), (131.274, 70.118)
            // Blue: (141.990, 85.512), (94.577, 88.443), (131.274, 73.882)
            SecondIntaketoGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(141.990, 85.512),
                                    new Pose(94.577, 88.443),
                                    new Pose(131.274, 73.882)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Red: (127.274, 70.118, 0°) → (96.8, 96.770, 41°)
            // Blue: (127.274, 73.882, 0°) → (96.8, 47.230, -41°)
            GateToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.274, 73.882),
                                    new Pose(96.8, 47.22966850828729)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-41))
                    .build();

            // Red control points: (85.108, 86.030), (65.505, 24.728), (136.025, 32.306)
            // Blue: (85.108, 57.970), (65.505, 119.272), (136.025, 111.694)
            shootToThirdIntake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.108, 57.970),
                                    new Pose(65.505, 119.272),
                                    new Pose(136.025, 111.694)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-41), Math.toRadians(0))
                    .build();

            // Red: (136.025, 36.306, 0°) → (88.227, 106.951, 25°)
            // Blue: (136.025, 107.694, 0°) → (88.227, 37.049, -25°)
            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(136.025, 107.694),
                                    new Pose(88.227, 37.049)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-25))
                    .build();
        }
    }
}