package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "April Tag Test", group = "Test")
public class aprilTagTest extends LinearOpMode {

    // ================= ENUMS =================
    enum BallColor { GREEN, PURPLE, NONE }
    enum ShootState { IDLE, FIRE_1, FIRE_2, FIRE_3 }

    // ================= HARDWARE =================
    private DcMotorEx outtake;
    private Servo kick1, kick2, kick3;
    private ColorSensor[] colorSensors = new ColorSensor[3];
    private Limelight3A limelight;

    // ================= CONSTANTS =================
    private static final double KICK_REST = 0.125;
    private static final double KICK_FIRE = 0.75;
    private static final long KICK_TIME_MS = 140;

    // ================= PATTERNS =================
    private final BallColor[] PATTERN_GPP = {
            BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE
    };
    private final BallColor[] PATTERN_PGP = {
            BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE
    };
    private final BallColor[] PATTERN_PPG = {
            BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN
    };

    // ================= STATE =================
    private BallColor[] detectedPattern = new BallColor[3];
    private BallColor[] lockedTargetPattern = null;
    private int ballCount = 0;

    private ShootState shootState = ShootState.IDLE;
    private long stateStartTime = 0;

    private final float[] hsv = new float[3];

    @Override
    public void runOpMode() {

        // ---- OUTTAKE ----
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorEx.Direction.REVERSE);

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

        // ---- LIMELIGHT ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            // ===== OUTTAKE ALWAYS ON =====
            outtake.setVelocity(1400);

            // ===== READ COLOR SENSORS =====
            ballCount = 0;
            for (int i = 0; i < 3; i++) {
                BallColor c = detectColor(colorSensors[i]);
                detectedPattern[i] = c;
                if (c != BallColor.NONE) ballCount++;
            }

            // ===== READ APRILTAG (ONLY IF NOT LOCKED) =====
            if (lockedTargetPattern == null) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                    if (fids != null && !fids.isEmpty()) {
                        int id = fids.get(0).getFiducialId();
                        if (id == 21) lockedTargetPattern = PATTERN_GPP;
                        if (id == 22) lockedTargetPattern = PATTERN_PGP;
                        if (id == 23) lockedTargetPattern = PATTERN_PPG;
                    }
                }
            }

            // ===== START SHOOTING =====
            if (shootState == ShootState.IDLE &&
                    gamepad1.right_trigger > 0.2 &&
                    lockedTargetPattern != null &&
                    ballCount == 3 &&
                    patternMatches(lockedTargetPattern)) {

                shootState = ShootState.FIRE_1;
                stateStartTime = System.currentTimeMillis();
            }

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
                        shootState = ShootState.IDLE;
                        lockedTargetPattern = null;
                    }
                    break;
            }

            telemetry.addData("Detected", detectedPattern[0] + " " +
                    detectedPattern[1] + " " + detectedPattern[2]);
            telemetry.addData("LockedPattern", lockedTargetPattern);
            telemetry.addData("ShootState", shootState);
            telemetry.update();
        }
    }

    // ================= HELPERS =================

    private BallColor detectColor(ColorSensor sensor) {
        Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsv);
        float h = hsv[0], s = hsv[1], v = hsv[2];

        if (s < 0.25 || v < 0.15) return BallColor.NONE;
        if ((h <= 25 || h >= 330)) return BallColor.NONE; // reject red
        if (h >= 95 && h <= 145) return BallColor.GREEN;
        if (h >= 240 && h <= 340) return BallColor.PURPLE;

        return BallColor.NONE;
    }

    private boolean patternMatches(BallColor[] target) {
        for (int i = 0; i < 3; i++) {
            if (detectedPattern[i] != target[i]) return false;
        }
        return true;
    }
}
