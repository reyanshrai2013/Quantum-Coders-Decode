package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RED SIDE TELEOP", group = "StarterBot")
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

    private final double RED = 0.277, ORANGE = 0.333, GREEN = 0.500, WHITE = 1;
    private double limelightAimed = 0;

    @Override
    public void init() {

        light = hardwareMap.get(Servo.class, "light");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rb");

        intake = hardwareMap.get(DcMotor.class, "intake");
        rightFeeder = hardwareMap.get(DcMotor.class, "feed");

        launcherRight = hardwareMap.get(DcMotorEx.class, "launch");
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launch1");

        // Directions (MATCH AUTO)
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFeeder.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setDirection(DcMotor.Direction.FORWARD);

        // Zero power behavior
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        rightFeeder.setZeroPowerBehavior(BRAKE);

        // Launcher PID
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients coeffs = new PIDFCoefficients(60, 0, 0, 12);
        launcherRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(1);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad2.left_bumper) {
            gamepad1.rumbleBlips(1);
        }
        telemetry.addData(">", "is reaysnh  RUMBLING? %s\n", gamepad1.isRumbling() ? "YES" : "no" );
        LLResult r = limelight.getLatestResult();
        telemetry.addData("Valid", r != null && r.isValid());
        telemetry.addData("tx", r == null ? "null" : r.getTx());
        telemetry.addData("ta", r == null ? "null" : r.getTa());
        telemetry.update();


        // ===== DRIVE CONTROL =====
        if (gamepad2.right_bumper) {
            autoTrackAprilTag();
        } else {
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
                }

                else if (gamepad1.x){
                    launcherTargetVelocity = 1500;
                    setLauncherVelocity(launcherTargetVelocity);
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                setLauncherVelocity(launcherTargetVelocity);

                if (Math.abs(launcherRight.getVelocity() - launcherTargetVelocity)
                        < VELOCITY_TOLERANCE) {
                    launchState = LaunchState.LAUNCH;
                }

                if (gamepad1.b) launchState = LaunchState.IDLE;
                break;

            case LAUNCH:
                setLauncherVelocity(launcherTargetVelocity);

                if (gamepad1.b) {
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
            setLauncherVelocity(launcherTargetVelocity);
        }

        if (lt && !lastLT) {
            launcherTargetVelocity = Math.max(launcherTargetVelocity - LAUNCHER_STEP, LAUNCHER_MIN_ADJUST);
            setLauncherVelocity(launcherTargetVelocity);
        }

        lastLB = lb;
        lastLT = lt;

        // ===== FEEDER =====
        if (gamepad1.right_bumper) {
            rightFeeder.setPower(0.75);
        } else if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) {
            rightFeeder.setPower(-0.55);
        } else if (gamepad1.right_trigger > 0.1) {
            rightFeeder.setPower(1);
        } else {
            rightFeeder.setPower(0);
        }

        // ===== INTAKE =====
        if (gamepad1.a) intake.setPower(1.0);
        else intake.setPower(0);

        // ===== LIGHT =====
        double vel = launcherLeft.getVelocity();

        if (Math.abs(vel) > 10) light.setPosition(RED);
        else if (limelightAimed == 1) light.setPosition(GREEN);
        else if (limelightAimed == 2) light.setPosition(ORANGE);
        else light.setPosition(WHITE);

        telemetry.update();
    }

    private void autoTrackAprilTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && !Double.isNaN(result.getTx())) {

            double tx = result.getTx();
            double rotationPower = tx * ROTATION_KP;

            if (Math.abs(tx) < TARGET_TOLERANCE) {
                rotationPower = 0;
                limelightAimed = 1;
            } else {
                if (Math.abs(rotationPower) < MIN_ROTATION_POWER)
                    rotationPower = Math.signum(rotationPower) * MIN_ROTATION_POWER;

                rotationPower = Math.max(-MAX_ROTATION_POWER,
                        Math.min(MAX_ROTATION_POWER, rotationPower));

                limelightAimed = 2;
            }

            mecanumDrive(
                    -gamepad2.left_stick_y,
                    gamepad2.left_stick_x,
                    rotationPower
            );

        } else {
            // SEARCH MODE — slow spin until tag found
            mecanumDrive(
                    -gamepad2.left_stick_y,
                    gamepad2.left_stick_x,
                    0.18
            );
            limelightAimed = 0;
        }
    }


    private void setLauncherVelocity(double v) {
        launcherRight.setVelocity(v);
        launcherLeft.setVelocity(v);
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double lf = (forward + strafe + rotate) / denominator;
        double rf = (forward - strafe - rotate) / denominator;
        double lb = (forward - strafe + rotate) / denominator;
        double rb = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }
}
