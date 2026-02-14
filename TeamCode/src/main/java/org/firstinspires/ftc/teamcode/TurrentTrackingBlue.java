package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TurrentTracking_Blue", group = "Production")
public class TurrentTrackingBlue extends LinearOpMode {

    private final int TARGET_ID = 20;

    private static double HORIZONTAL_OFFSET = 5.8;

    // 6000 RPM motors (~2800 ticks/sec max)
    private double START_RPM = 1800.0;
    private double RPM_PER_INCH = 8.0;

    private double kP_Turret = 0.040, kD_Turret = 0.080;
    private double previousError = 0, smoothedTx = 0, filterWeight = 0.45;

    private DcMotorEx leftFront, rightFront, leftBack, rightBack, turret;
    private DcMotorEx outtakeL, outtakeR;
    private DcMotor intake;

    private Servo[] kickers = new Servo[4];
    private Limelight3A limelight;
    private final ColorSensor[] sensors = new ColorSensor[7];
    private RevBlinkinLedDriver led;

    private static final double[] REST = {0.10, 0.00, 1.00, 1.00};
    private static final double[] FIRE = {0.70, 0.95, 0.45, 0.40};

    private final boolean[] firing = new boolean[4];
    private final long[] fireEndMs = new long[4];

    private boolean prevDpadLeft = false;
    private boolean prevA2 = false, prevY2 = false;
    private boolean intakeCancelStop = false;

    private BallColor[] cachedColors = new BallColor[7];
    private double lastKnownTargetVelocity = 2000;
    private boolean tagVisible = false;

    @Override
    public void runOpMode() {

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");

        intake = hardwareMap.get(DcMotor.class, "intake");

        // Brake mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse ONE flywheel motor
        outtakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Starter PIDF values
        outtakeL.setVelocityPIDFCoefficients(50, 0, 5, 12);
        outtakeR.setVelocityPIDFCoefficients(50, 0, 5, 12);

        for (int i = 0; i < 4; i++)
            kickers[i] = hardwareMap.get(Servo.class, "kick" + (i + 1));

        for (int i = 0; i < 7; i++)
            sensors[i] = hardwareMap.get(ColorSensor.class, "color" + (i + 1));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            long now = System.currentTimeMillis();

            // ---------------- VISION ----------------
            LLResult result = limelight.getLatestResult();
            tagVisible = false;

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                    if (f.getFiducialId() == TARGET_ID) {

                        tagVisible = true;
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

                        double distInches =
                                Math.abs(f.getTargetPoseRobotSpace()
                                        .getPosition().z * 39.37);

                        lastKnownTargetVelocity =
                                clamp(START_RPM + (distInches * RPM_PER_INCH),
                                        1500,
                                        2700);

                        double rawTx = f.getTargetXDegrees() + HORIZONTAL_OFFSET;
                        smoothedTx = (rawTx * filterWeight)
                                + (smoothedTx * (1.0 - filterWeight));

                        break;
                    }
                }
            }

            if (!tagVisible) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            // ---------------- FLYWHEEL ----------------
            if (gamepad2.right_trigger > 0.3) {

                outtakeL.setVelocity(0);
                outtakeR.setVelocity(0);

            } else {

                outtakeL.setVelocity(lastKnownTargetVelocity);
                outtakeR.setVelocity(lastKnownTargetVelocity);
            }

            // ---------------- DRIVE ----------------
            double scale = gamepad1.right_bumper ? 0.4 : 1.0;
            driveRobot(-gamepad1.left_stick_y * scale,
                    gamepad1.left_stick_x * scale,
                    gamepad1.right_stick_x * scale);

            // ---------------- TURRET ----------------
            if (gamepad1.left_trigger > 0.2 && tagVisible) {
                double derivative = (smoothedTx - previousError);
                turret.setPower(clamp(
                        (smoothedTx * kP_Turret)
                                + (derivative * kD_Turret)
                                + (Math.signum(smoothedTx) * 0.16),
                        -0.8,
                        0.8));
                previousError = smoothedTx;
            } else {
                turret.setPower(0);
                previousError = 0;
            }
        }
    }

    private void driveRobot(double y, double x, double rx) {
        double lf = y + x + rx;
        double rf = y - x - rx;
        double lb = y - x + rx;
        double rb = y + x - rx;
        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.abs(lb))));
        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    enum BallColor { GREEN, PURPLE, WHITE, NONE }
}
