package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public  class PathsForNineBallAuto {
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

        public PathsForNineBallAuto(Follower follower) {

            Wait1 = 1750;

            startToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(125.6945320197044, 122.38384729064042),
                                    new Pose(93.84280662983427, 85)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                    .build();

            shootToFirstInkate = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(93.84280662983427, 85),
                                    new Pose(132.19211822660097, 84.84236453201973)
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
                                    new Pose(132.547, 75.621),
                                    new Pose(96.8, 96.77)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(31))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.8, 96.77),
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
                                    new Pose(131.274, 70.118),
                                    new Pose(96.8, 96.77033149171271)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))
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
