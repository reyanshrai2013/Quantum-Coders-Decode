package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public  class PathsForBlueCloseNineBallAuto {
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

    public PathsForBlueCloseNineBallAuto(Follower follower) {

        Wait1 = 1750;

        startToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125.6945320197044, 122.38384729064042).mirror(),
                                new Pose(95.84280662983427, 87).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-36), Math.toRadians(180-45))
                .build();

        shootToFirstInkate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(95.84280662983427, 87).mirror(),
                                new Pose(132.19211822660097, 84.84236453201973).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        firstIntakeToGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.19211822660097, 84.84236453201973).mirror(),
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

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(96.8, 96.77033149171271).mirror(),
                                new Pose(88.227, 106.951).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(-120))
                .build();
    }
}
