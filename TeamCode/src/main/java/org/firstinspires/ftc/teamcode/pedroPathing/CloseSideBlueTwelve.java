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
    private DcMotorEx launcher2 = null;   // ADDED SECOND LAUNCHER
    private DcMotorEx intake = null;
    private DcMotorEx feed = null;

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launch");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launch1");   // ADDED

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        feed = hardwareMap.get(DcMotorEx.class, "feed");

        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher2.setDirection(DcMotorEx.Direction.FORWARD);       // ADDED
        feed.setDirection(DcMotorEx.Direction.REVERSE);


        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
        launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);   // ADDED

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-125.6945320197044, 122.38384729064042, Math.toRadians(180-36)));
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // NEW HELPER — replaces all launcher.setVelocity()
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
                    feed.setPower(1);
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
                    intake.setPower(1.0);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    follower.followPath(paths.shootToThirdIntake, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1090);
                    pathStarted = false;
                    pathState = 13;
                }
                break;



            case 13:
                if (!pathStarted) {
                    setLauncherVelocity(1090);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStartTime = System.currentTimeMillis();
                    pathStarted = true;
                    waitStarted = true;
                }

                if (!follower.isBusy()) {
                    setLauncherVelocity(1090);
                    feed.setPower(0);
                    intake.setPower(0);
                    waitStarted = false;
                    pathStarted = false;
                    pathState = 14;
                }
                break;

            case 15:
                if (!pathStarted) {
                    setLauncherVelocity(1090);
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


            startToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(125.6945320197044, 122.38384729064042).mirror(),
                                    new Pose(95.84280662983427, 91).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-36), Math.toRadians(180-47.5))
                    .build();

            shootToFirstInkate = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(95.84280662983427, 91).mirror(),
                                    new Pose(130.19211822660097, 80.84236453201973).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            firstIntakeToGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.19211822660097, 80.84236453201973).mirror(),
                                    new Pose(108.685, 75.453).mirror(),
                                    new Pose(130.547, 75.621).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            gateToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(130.547, 75.621).mirror(),
                                    new Pose(96.8, 96.77).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(131))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.8, 96.77).mirror(),
                                    new Pose(97.419, 58.596).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(180))
                    .build();

            Path7ToSecondIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(97.419, 59.596).mirror(),
                                    new Pose(139.99014778325125, 58.48768472906403).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SecondIntaketoGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(139.990, 58.488).mirror(),
                                    new Pose(94.577, 55.557).mirror(),
                                    new Pose(129.274, 72.618).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            GateToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(129.274, 72.618).mirror(),
                                    new Pose(96.8, 96.77033149171271).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(131))
                    .build();

            shootToThirdIntake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.8, 96.77033149171271).mirror(),
                                    new Pose(65.505, 24.728).mirror(),
                                    new Pose(136.025, 32.306).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(180))
                    .build();



            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(136.025, 36.306).mirror(),
                                    new Pose(89.227, 109.951).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180-29))
                    .build();
        }
    }
}

