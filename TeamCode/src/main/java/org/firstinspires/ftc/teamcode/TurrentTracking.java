package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;

@TeleOp(name = "Turret_AutoMatch_V58", group = "Production")
public class TurrentTracking extends LinearOpMode {

    private final int TARGET_ID = 24;
    private static double HORIZONTAL_OFFSET = 6.0;

    private double kP_Turret = 0.040, kD_Turret = 0.080;
    private double previousError = 0, smoothedTx = 0, filterWeight = 0.45;
    private double lastTurretPower = 0;

    private DcMotorEx leftFront, rightFront, leftBack, rightBack, turret, outtake;
    private DcMotor intake;
    private Servo[] kickers = new Servo[4];
    private Limelight3A limelight;
    private final ColorSensor[] sensors = new ColorSensor[7];

    private static final double[] REST = {0.10, 0.00, 1.00, 1.00};
    private static final double[] FIRE = {0.70, 0.95, 0.45, 0.40};

    private final boolean[] firing = new boolean[4];
    private final long[] fireEndMs = new long[4];
    private boolean prevX = false, prevB = false;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotor.class, "intake");

        kickers[0] = hardwareMap.get(Servo.class, "kick1");
        kickers[1] = hardwareMap.get(Servo.class, "kick2");
        kickers[2] = hardwareMap.get(Servo.class, "kick3");
        kickers[3] = hardwareMap.get(Servo.class, "kick4");

        for (int i = 0; i < 7; i++) sensors[i] = hardwareMap.get(ColorSensor.class, "color" + (i + 1));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();
            LLResult result = limelight.getLatestResult();

            boolean id24Visible = false;
            double targetTy = 0;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult f : fiducials) {
                    if (f.getFiducialId() == TARGET_ID) {
                        id24Visible = true;
                        targetTy = f.getTargetYDegrees();
                        double rawTx = f.getTargetXDegrees() + HORIZONTAL_OFFSET;
                        smoothedTx = (rawTx * filterWeight) + (smoothedTx * (1.0 - filterWeight));
                        break;
                    }
                }
            }

            driveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // Turret Tracking
            if (gamepad1.left_trigger > 0.2) {
                if (id24Visible) {
                    double derivative = (smoothedTx - previousError);
                    double pidOutput = (smoothedTx * kP_Turret) + (derivative * kD_Turret);
                    previousError = smoothedTx;
                    lastTurretPower = clamp(pidOutput + (Math.signum(pidOutput) * 0.16), -0.80, 0.80);
                    turret.setPower(lastTurretPower);
                } else {
                    lastTurretPower *= 0.90;
                    turret.setPower(lastTurretPower);
                }
            } else {
                turret.setPower(gamepad2.right_bumper ? 0.45 : (gamepad2.left_bumper ? -0.45 : 0));
                lastTurretPower = 0;
                previousError = 0;
            }

            // Shooter Velocity
            if (gamepad2.right_trigger > 0.2) {
                double targetVelocity;
                if (targetTy >= -5.0) targetVelocity = 1300;
                else if (targetTy <= -11.0) targetVelocity = 1400;
                else targetVelocity = 1300 + (targetTy + 5.0) * (100.0 / -6.0);

                outtake.setVelocity(targetVelocity);

                if (gamepad2.x && !prevX) {
                    int slot = findKickerByColor(BallColor.PURPLE);
                    if (slot != -1) requestKick(slot, now);
                }
                if (gamepad2.b && !prevB) {
                    int slot = findKickerByColor(BallColor.GREEN);
                    if (slot != -1) requestKick(slot, now);
                }
            } else { outtake.setVelocity(0); }

            intake.setPower(gamepad1.a ? 1.0 : (gamepad1.b ? -1.0 : 0));
            updateKickers(now);

            telemetry.addData("Ty", targetTy);
            telemetry.addData("Velocity", outtake.getVelocity());

            // Color detection telemetry
            for (int i = 0; i < 7; i++) {
                BallColor color = detectColor(sensors[i]);
                telemetry.addData("Color Sensor " + (i+1), color);
            }

            telemetry.update();

            prevX = gamepad2.x; prevB = gamepad2.b;
        }
    }

    private void driveRobot(double y, double x, double rx) {
        double lf = y + x + rx, rf = y - x - rx, lb = y - x + rx, rb = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
        leftFront.setPower(lf/max); rightFront.setPower(rf/max);
        leftBack.setPower(lb/max); rightBack.setPower(rb/max);
    }

    private void requestKick(int id, long now) {
        if (id >= 0 && id < 4 && !firing[id]) {
            firing[id] = true; fireEndMs[id] = now + 400;
        }
    }

    private void updateKickers(long now) {
        for (int i = 0; i < 4; i++) {
            if (firing[i]) {
                kickers[i].setPosition(FIRE[i]);
                if (now > fireEndMs[i]) firing[i] = false;
            } else { kickers[i].setPosition(REST[i]); }
        }
    }

    private int findKickerByColor(BallColor target) {
        if (detectColor(sensors[0]) == target || detectColor(sensors[1]) == target) return 0;
        if (detectColor(sensors[2]) == target || detectColor(sensors[3]) == target) return 1;
        if (detectColor(sensors[4]) == target || detectColor(sensors[5]) == target) return 2;
        if (detectColor(sensors[6]) == target) return 3;
        return -1;
    }

    private BallColor detectColor(ColorSensor s) {
        int r = s.red();
        int g = s.green();
        int b = s.blue();
        if (r + g + b < 20) return BallColor.NONE;

        float[] hsvLocal = new float[3];
        android.graphics.Color.RGBToHSV(r, g, b, hsvLocal);
        float h = hsvLocal[0];
        float sValue = hsvLocal[1];  // Saturation value

        telemetry.addData("Sensor Red", r);
        telemetry.addData("Sensor Green", g);
        telemetry.addData("Sensor Blue", b);
        telemetry.addData("HSV Hue", h);
        telemetry.addData("HSV Saturation", sValue);

        // Check for valid saturation level
        if (sValue < 0.15) return BallColor.NONE;  // Use 0.15 as the new threshold for saturation

        if (h >= 30 && h <= 150) {
            telemetry.addData("Detected Color", "GREEN");
            return BallColor.GREEN;
        }
        if (h >= 240 && h <= 360) {
            telemetry.addData("Detected Color", "PURPLE");
            return BallColor.PURPLE;
        }

        telemetry.addData("Detected Color", "NONE");
        return BallColor.NONE;
    }

    private double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }
    enum BallColor { GREEN, PURPLE, NONE }
}
