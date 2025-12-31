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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "April Tag Test", group = "Test")
public class aprilTagTest extends LinearOpMode {

    private static final Object TICKS_PER_REV = 3500;

    // ===================== ENUMS =====================
    enum BallColor { GREEN, PURPLE, NONE }
    enum ShootState { IDLE, FIRE_1, FIRE_2, FIRE_3, DONE }

    // ===================== DRIVE =====================
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // ===================== TURRET =====================
    private DcMotorEx turret;

    // ===================== SHOOTER =====================
    private DcMotorEx outtake;

    // ===================== KICKERS =====================
    private Servo kick1, kick2, kick3;
    private static final double KICK_REST = 0.1;
    private static final double KICK_FIRE = 0.75;

    // ===================== COLOR SENSORS =====================
    private ColorSensor[] colorSensors = new ColorSensor[3];
    private final float[] hsv = new float[3];

    // ===================== PATTERN =====================
    private BallColor[] detectedPattern = new BallColor[3];
    private BallColor[] activeTargetPattern = null;
    private int ballCount = 0;

    private final BallColor[] PATTERN_PPG = {
            BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN
    };
    private final BallColor[] PATTERN_GPP = {
            BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE
    };
    private final BallColor[] PATTERN_PGP = {
            BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE
    };

    // ===================== SHOOT STATE =====================
    private ShootState shootState = ShootState.IDLE;
    private long stateStartTime = 0;
    private static final long KICK_TIME_MS = 140;

    // ===================== IMU =====================
    private IMU imu;
    private static final double HEADING_SIGN = -1.0;
    private double headingFiltered = 0.0;

    // ===================== LIMELIGHT =====================
    private Limelight3A limelight;
    private static final double LIMELIGHT_TOL = 1.0;
    private double txFiltered = 0.0;
    private boolean haveTargetCached = false;

    // ===================== RPM =====================
    private static final double FAR_RPM = 3500;
    private static final double CLOSE_RPM = 2500;

    // ==================================================
    @Override
    public void runOpMode() {

        // ---- DRIVE ----
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // ---- TURRET ----
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---- OUTTAKE ----
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- KICKERS ----
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");

        kick1.setDirection(Servo.Direction.FORWARD);
        kick2.setDirection(Servo.Direction.REVERSE);
        kick3.setDirection(Servo.Direction.REVERSE);

        kick1.setPosition(KICK_REST);
        kick2.setPosition(KICK_REST);
        kick3.setPosition(KICK_REST);

        // ---- COLOR SENSORS ----
        colorSensors[0] = hardwareMap.get(ColorSensor.class, "color1");
        colorSensors[1] = hardwareMap.get(ColorSensor.class, "color2");
        colorSensors[2] = hardwareMap.get(ColorSensor.class, "color3");

        // ---- IMU ----
        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        // ---- LIMELIGHT ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            colorkicktest.BallColor targetColor = null;
            if (gamepad1.x) targetColor = colorkicktest.BallColor.GREEN;
            else if (gamepad1.b) targetColor = colorkicktest.BallColor.PURPLE;
            outtake.setVelocity(1400);
            // ================= LIMELIGHT + PATTERN SELECT =================
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                if (fids != null) {
                    for (LLResultTypes.FiducialResult fid : fids) {
                        switch (fid.getFiducialId()) {
                            case 21: activeTargetPattern = PATTERN_GPP; break;
                            case 22: activeTargetPattern = PATTERN_PGP; break;
                            case 23: activeTargetPattern = PATTERN_PPG; break;
                        }
                        txFiltered = fid.getTargetXDegrees();
                        haveTargetCached = true;
                        break;
                    }
                }
            }

            // ================= BUILD BALL PATTERN =================


            boolean aligned = haveTargetCached && Math.abs(txFiltered) < LIMELIGHT_TOL;
            boolean shooterReady = gamepad1.right_trigger > 0.2;

            // ================= LOCK PATTERN =================
            if (shootState == ShootState.IDLE && shooterReady &&
                    ballCount == 3 &&
                    activeTargetPattern != null &&
                    patternMatches(activeTargetPattern)) {

                shootState = ShootState.FIRE_1;
                stateStartTime = System.currentTimeMillis();
            }

            // ================= SHOOT STATE MACHINE =================
            long now = System.currentTimeMillis();

            switch (shootState) {
                case FIRE_1:
                    kick1.setPosition(KICK_FIRE);
                    if (now - stateStartTime > KICK_TIME_MS) {
                        kick1.setPosition(KICK_REST);
                        shootState = ShootState.FIRE_2;
                        stateStartTime = now;
                    }
                    break;

                case FIRE_2:
                    kick2.setPosition(KICK_FIRE);
                    if (now - stateStartTime > KICK_TIME_MS) {
                        kick2.setPosition(KICK_REST);
                        shootState = ShootState.FIRE_3;
                        stateStartTime = now;
                    }
                    break;

                case FIRE_3:
                    kick3.setPosition(KICK_FIRE);
                    if (now - stateStartTime > KICK_TIME_MS) {
                        kick3.setPosition(KICK_REST);
                        shootState = ShootState.DONE;
                    }
                    break;

                case DONE:
                    ballCount = 0;
                    shootState = ShootState.IDLE;
                    break;
            }

            // ================= TELEMETRY =================
            telemetry.addData("ShootState", shootState);
            telemetry.addData("Detected",
                    detectedPattern[0] + " " +
                            detectedPattern[1] + " " +
                            detectedPattern[2]);
            telemetry.addData("ActivePattern", activeTargetPattern);
            telemetry.update();
        }
    }

    // ===================== HELPERS =====================

    private BallColor detectColor(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        Color.RGBToHSV(r, g, b, hsv);

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        // Relaxed thresholds for low light / purple detection
        if (sat < 0.25 || val < 0.15) return BallColor.NONE;

        // Reject red
        if ((hue >= 0 && hue <= 25) || (hue >= 330 && hue <= 360)) return BallColor.NONE;

        // Green threshold
        if (hue >= 95 && hue <= 145) return BallColor.GREEN;

        // Purple threshold widened aggressively
        if (hue >= 240 && hue <= 340) return BallColor.PURPLE;

        return BallColor.NONE;
    }

    private boolean patternMatches(BallColor[] target) {
        for (int i = 0; i < 3; i++) {
            if (detectedPattern[i] != target[i]) return false;
        }
        return true;
    }
}
