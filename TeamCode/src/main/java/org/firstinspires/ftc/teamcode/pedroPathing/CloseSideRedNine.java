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

@Autonomous(name = "CLOSE SIDE RED NINE", group = "Autonomous")
@Configurable
public class CloseSideRedNine extends OpMode {

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
        follower.setStartingPose(new Pose(125.6945320197044, 122.38384729064042, Math.toRadians(36)));
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
                    setLauncherVelocity(1175);
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
                    setLauncherVelocity(1175);
                    launcherStartTime = System.currentTimeMillis();
                    pathStarted = true;
                }

                if (System.currentTimeMillis() - launcherStartTime >= 500 && feed.getPower() == 0) {
                    feed.setPower(0.6);
                    intake.setPower(1.0);
                }

                if (waitStarted && System.currentTimeMillis() - waitStartTime >= paths.Wait1) {
                    setLauncherVelocity(1175);
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
                    setLauncherVelocity(1175);
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
                    setLauncherVelocity(1175);
                    follower.followPath(paths.gateToShoot, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1175);
                    pathStarted = false;
                    pathState = 6;
                }
                break;

            case 6:
                if (!pathStarted) {
                    setLauncherVelocity(1175);
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
                    setLauncherVelocity(1175);
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
                    setLauncherVelocity(1175);
                    follower.followPath(paths.Path7, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1175);
                    pathStarted = false;
                    pathState = 8;
                }
                break;

            case 8:
                if (!pathStarted) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1175);
                    follower.followPath(paths.Path7ToSecondIntake, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1175);
                    pathStarted = false;
                    pathState = 85;
                }
                break;

            case 85:
                if (!pathStarted) {
                    intake.setPower(0.5);
                    feed.setPower(0);
                    setLauncherVelocity(1175);
                    follower.followPath(paths.SecondIntaketoGate, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1175);
                    pathStarted = false;
                    pathState = 9;
                }
                break;

            case 9:
                if (!pathStarted) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1175);
                    follower.followPath(paths.GateToShoot, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(0);
                    feed.setPower(0);
                    setLauncherVelocity(1175);
                    pathStarted = false;
                    pathState = 10;
                }
                break;

            case 10:
                if (!pathStarted) {
                    setLauncherVelocity(1175);
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
                    setLauncherVelocity(1150);
                    follower.followPath(paths.Path9, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(1);
                    feed.setPower(0);
                    setLauncherVelocity(1150);
                    pathStarted = false;
                    pathState = 16;
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
                                    new Pose(125.6945320197044, 122.38384729064042),
                                    new Pose(96.84280662983427, 96.77033149171271)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                    .build();

            shootToFirstInkate = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.970, 82.833),
                                    new Pose(132.19211822660097, 82.84236453201973)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            firstIntakeToGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(132.19211822660097, 84.84236453201973),
                                    new Pose(108.685, 75.453),
                                    new Pose(132.547, 75.621)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            gateToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.547, 72.621),
                                    new Pose(96.8, 96.77)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(34))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(85.675, 85.611),
                                    new Pose(100.419, 58.596)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0))
                    .build();

            Path7ToSecondIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.419, 59.596),
                                    new Pose(141.99014778325125, 58.48768472906403)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
            SecondIntaketoGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(141.990, 58.488),
                                    new Pose(94.577, 55.557),
                                    new Pose(131.274, 70.118)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GateToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.274, 70.118),
                                    new Pose(96.8, 96.77033149171271)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(41))
                    .build();





            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.8, 96.77033149171271),
                                    new Pose(88.227, 106.951)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(-30))
                    .build();
        }
    }
}
