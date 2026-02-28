package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.pedroPathing.Importantthingsithasrizztrust.LauncherPIDF.coeffs;
import static org.firstinspires.ftc.teamcode.pedroPathing.Importantthingsithasrizztrust.daperfectlighttune.*;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "BLUE SIDE TELEOP", group = "Teleop")
public class BlueTeleop extends OpMode {

    final double LAUNCHER_TARGET_VELOCITY = 1200;
    final double LAUNCHER_MAX_VELOCITY = 6000;
    final double LAUNCHER_MIN_ADJUST = 0;
    final double LAUNCHER_STEP = 50;

    final double VELOCITY_TOLERANCE = 75;

    // Limelight tracking constants
    final double ROTATION_KP = 0.05;
    final double TARGET_TOLERANCE = 2.0;
    final double MIN_ROTATION_POWER = 0.15;
    final double MAX_ROTATION_POWER = 0.5;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor intake, rightFeeder;
    private DcMotorEx launcherRight, launcherLeft;

    private Servo light;
    private Limelight3A limelight;

    private enum LaunchState {IDLE, SPIN_UP, LAUNCH}
    private LaunchState launchState = LaunchState.IDLE;

    private double launcherTargetVelocity = 0;
    private boolean lastLB = false, lastLT = false;

    private int limelightAimed = 0; // 0=no target, 1=locked, 2=tracking

    private boolean hasRumbledOnLock = false;

    @Override
    public void init() {
        light = hardwareMap.get(Servo.class, "light");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rb");

        intake        = hardwareMap.get(DcMotor.class, "intake");
        rightFeeder   = hardwareMap.get(DcMotor.class, "feed");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launch");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launch1");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        rightFeeder.setZeroPowerBehavior(BRAKE);
        launcherLeft.setZeroPowerBehavior(BRAKE);
        launcherRight.setZeroPowerBehavior(BRAKE);

        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(1);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad2.left_bumper) gamepad1.rumbleBlips(1);

        LLResult r = limelight.getLatestResult();
        telemetry.addData("Valid", r != null && r.isValid());
        telemetry.addData("tx", r == null ? "null" : r.getTx());
        telemetry.addData("Aimed", limelightAimed == 1 ? "LOCKED" : limelightAimed == 2 ? "TRACKING" : "NO TARGET");
        telemetry.addData(
                "Launcher At Speed",
                Math.abs(launcherRight.getVelocity() - launcherTargetVelocity) < VELOCITY_TOLERANCE
        );
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Target Velocity", launcherTargetVelocity);
        telemetry.addData("Actual Velocity", launcherRight.getVelocity());

        // ===== DRIVE CONTROL =====
        if (gamepad2.right_bumper) {
            autoTrackAprilTag();
        } else {
            hasRumbledOnLock = false;
            limelightAimed = 0;
            mecanumDrive(
                    -gamepad2.left_stick_y,
                    gamepad2.left_stick_x,
                    gamepad2.right_stick_x
            );
        }

        // ===== LAUNCHER STATE MACHINE =====
        switch (launchState) {
            case IDLE:
                setLauncherVelocity(0);
                rightFeeder.setPower(0);
                if (gamepad1.y) {
                    launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY;
                    setLauncherVelocity(launcherTargetVelocity);
                    launchState = LaunchState.SPIN_UP;
                } else if (gamepad1.x) {
                    launcherTargetVelocity = 1500;
                    setLauncherVelocity(launcherTargetVelocity);
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                setLauncherVelocity(launcherTargetVelocity);
                if (Math.abs(launcherRight.getVelocity() - launcherTargetVelocity) < VELOCITY_TOLERANCE) {
                    launchState = LaunchState.LAUNCH;
                }
                // Only cancel if b is freshly pressed (avoid accidental cancel)
                if (gamepad1.b && !gamepad1.y && !gamepad1.x) {
                    setLauncherVelocity(-1000);
                    launchState = LaunchState.IDLE;
                }
                break;

            case LAUNCH:
                setLauncherVelocity(launcherTargetVelocity);
                if (gamepad1.b && !gamepad1.y && !gamepad1.x) {
                    rightFeeder.setPower(0);
                    launchState = LaunchState.IDLE;
                }
                break;
        }

        // ===== RPM ADJUST =====
        boolean lb = gamepad1.left_bumper;
        boolean lt = gamepad1.left_trigger > 0.1;
        if (lb && !lastLB) {
            launcherTargetVelocity = Math.min(launcherTargetVelocity + LAUNCHER_STEP, LAUNCHER_MAX_VELOCITY);
            if (launchState != LaunchState.IDLE) setLauncherVelocity(launcherTargetVelocity);
        }
        if (lt && !lastLT) {
            launcherTargetVelocity = Math.max(launcherTargetVelocity - LAUNCHER_STEP, LAUNCHER_MIN_ADJUST);
            if (launchState != LaunchState.IDLE) setLauncherVelocity(launcherTargetVelocity);
        }
        lastLB = lb;
        lastLT = lt;

        // ===== FEEDER =====
        if (launchState == LaunchState.LAUNCH) {
            if (gamepad1.right_bumper) {
                rightFeeder.setPower(0.75);
            } else if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) {
                rightFeeder.setPower(-0.55);
            } else if (gamepad1.right_trigger > 0.1) {
                rightFeeder.setPower(1);
            } else {
                rightFeeder.setPower(0);
            }
        } else if (launchState == LaunchState.IDLE) {
            // Still allow manual feeder control even when launcher is off
            if (gamepad1.right_bumper) {
                rightFeeder.setPower(0.75);
            } else if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) {
                rightFeeder.setPower(-0.55);
            } else if (gamepad1.right_trigger > 0.1) {
                rightFeeder.setPower(1);
            } else {
                rightFeeder.setPower(0);
            }
        }

        // ===== INTAKE =====
        intake.setPower(gamepad1.a ? 1.0 : 0);

        // ===== LIGHT (priority: launcher on > limelight locked > limelight tracking > idle) =====
        boolean launcherOn = launchState == LaunchState.SPIN_UP || launchState == LaunchState.LAUNCH;
        if (launcherOn)               light.setPosition(red);
        else if (limelightAimed == 1) light.setPosition(green);
        else if (limelightAimed == 2) light.setPosition(orange);
        else                          light.setPosition(white);

        // Single telemetry update at the end of loop
        telemetry.update();
    }

    private void autoTrackAprilTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && !Double.isNaN(result.getTx())) {
            double tx = result.getTx();

            if (Math.abs(tx) < TARGET_TOLERANCE) {
                // Perfectly aimed — stop rotating, rumble once
                limelightAimed = 1;
                if (!hasRumbledOnLock) {
                    gamepad2.rumbleBlips(2);
                    hasRumbledOnLock = true;
                }
                mecanumDrive(-gamepad2.left_stick_y, gamepad2.left_stick_x, 0);

            } else {
                // Tag visible but not centered — fine-tune rotation
                limelightAimed   = 2;
                hasRumbledOnLock = false;

                double rotationPower = tx * ROTATION_KP;
                if (Math.abs(rotationPower) < MIN_ROTATION_POWER)
                    rotationPower = Math.signum(rotationPower) * MIN_ROTATION_POWER;
                rotationPower = Math.max(-MAX_ROTATION_POWER, Math.min(MAX_ROTATION_POWER, rotationPower));

                mecanumDrive(-gamepad2.left_stick_y, gamepad2.left_stick_x, rotationPower);
            }

        } else {
            // No tag visible — hand full control back to driver, no spinning
            limelightAimed   = 0;
            hasRumbledOnLock = false;
            mecanumDrive(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
        }
    }

    private void setLauncherVelocity(double v) {
        launcherRight.setVelocity(v);
        launcherLeft.setVelocity(v);
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        leftFrontDrive.setPower((forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower((forward - strafe - rotate) / denominator);
        leftBackDrive.setPower((forward - strafe + rotate) / denominator);
        rightBackDrive.setPower((forward + strafe - rotate) / denominator);
    }
}