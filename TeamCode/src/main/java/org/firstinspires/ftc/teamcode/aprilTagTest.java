package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "aprilTagTest", group = "Test")
public class aprilTagTest extends LinearOpMode {

    enum BallColor { GREEN, PURPLE, NONE }

    private Servo[] kickers = new Servo[3];
    private ColorSensor[] sensors = new ColorSensor[6];
    private DcMotorEx outtake;
    private Limelight3A limelight;

    private static final double OUTTAKE_VELOCITY = 1300;

    private static final double[] KICK_FIRE = {0.7, 0.35, 0.45};
    private static final double[] KICK_REST = {0.1, 0.95, 1.0};

    // Timings (~1 second per ball)
    private static final long KICK_DELAY_MS = 350;         // Up / Down
    private static final long INTER_KICK_DELAY_MS = 300;   // Between kicks

    private static final BallColor[] PATTERN_PPG = { BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN };
    private static final BallColor[] PATTERN_PGP = { BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE };
    private static final BallColor[] PATTERN_GPP = { BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };

    private BallColor[] activePattern = null;

    private boolean shooting = false;
    private boolean canShoot = true;
    private int shootIndex = 0;
    private int shootState = 0;
    private long stateTime = 0;

    private int activeKicker = -1;
    private boolean[] kickerUsed = new boolean[3]; // track which kicker has already fired

    private final float[] hsv = new float[3];

    @Override
    public void runOpMode() {

        // ---- KICKERS ----
        kickers[0] = hardwareMap.get(Servo.class, "kick1");
        kickers[1] = hardwareMap.get(Servo.class, "kick2");
        kickers[2] = hardwareMap.get(Servo.class, "kick3");
        for (int i = 0; i < 3; i++) kickers[i].setPosition(KICK_REST[i]);

        // ---- COLOR SENSORS ----
        sensors[0] = hardwareMap.get(ColorSensor.class, "color1");
        sensors[1] = hardwareMap.get(ColorSensor.class, "color2");
        sensors[2] = hardwareMap.get(ColorSensor.class, "color4");
        sensors[3] = hardwareMap.get(ColorSensor.class, "color3");
        sensors[4] = hardwareMap.get(ColorSensor.class, "color6");
        sensors[5] = hardwareMap.get(ColorSensor.class, "color5");

        // ---- OUTTAKE ----
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- LIMELIGHT ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            outtake.setVelocity(OUTTAKE_VELOCITY);

            // ---------- APRILTAG ----------
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                if (fids != null) {
                    for (LLResultTypes.FiducialResult fid : fids) {
                        if (fid.getFiducialId() == 21) activePattern = PATTERN_GPP;
                        if (fid.getFiducialId() == 22) activePattern = PATTERN_PGP;
                        if (fid.getFiducialId() == 23) activePattern = PATTERN_PPG;
                    }
                }
            }

            // ---------- SHOOT LOGIC ----------
            if (activePattern != null) {

                if (gamepad1.x && canShoot) {

                    if (!shooting) {
                        shooting = true;
                        shootIndex = 0;
                        shootState = 0;
                        activeKicker = -1;
                        stateTime = System.currentTimeMillis();
                        for (int i = 0; i < 3; i++) kickerUsed[i] = false; // reset kicker usage
                    }

                    long now = System.currentTimeMillis();

                    if (shootIndex < 3) {

                        switch (shootState) {

                            case 0: // FIND + FIRE
                                activeKicker = findMatchingKicker(activePattern[shootIndex]);
                                if (activeKicker != -1) {
                                    kickers[activeKicker].setPosition(KICK_FIRE[activeKicker]);
                                    stateTime = now;
                                    shootState = 1;
                                }
                                break;

                            case 1: // WAIT UP
                                if (now - stateTime >= KICK_DELAY_MS) {
                                    kickers[activeKicker].setPosition(KICK_REST[activeKicker]);
                                    stateTime = now;
                                    shootState = 2;
                                }
                                break;

                            case 2: // WAIT DOWN + INTER-KICK WAIT
                                if (now - stateTime >= KICK_DELAY_MS + INTER_KICK_DELAY_MS) {
                                    shootIndex++;
                                    shootState = 0;
                                    activeKicker = -1;
                                }
                                break;
                        }

                    } else {
                        shooting = false;
                        canShoot = false;
                    }

                } else if (!gamepad1.x) {
                    canShoot = true;
                }
            }

            // ---------- TELEMETRY ----------
            telemetry.addData("Pattern",
                    activePattern == null ? "NONE" :
                            activePattern[0] + " " +
                                    activePattern[1] + " " +
                                    activePattern[2]);

            telemetry.addData("K1", detectColor(sensors[0]) + "," + detectColor(sensors[1]));
            telemetry.addData("K2", detectColor(sensors[2]) + "," + detectColor(sensors[3]));
            telemetry.addData("K3", detectColor(sensors[4]) + "," + detectColor(sensors[5]));

            telemetry.addData("Shoot Index", shootIndex);
            telemetry.addData("Shoot State", shootState);
            telemetry.addData("Active Kicker", activeKicker);

            telemetry.update();
        }
    }

    // ---------- HELPERS ----------

    private int findMatchingKicker(BallColor target) {
        for (int i = 0; i < 3; i++) {
            if (kickerUsed[i]) continue; // skip kickers already used
            if (detectColor(sensors[i * 2]) == target ||
                    detectColor(sensors[i * 2 + 1]) == target) {
                kickerUsed[i] = true; // mark this kicker as used
                return i;
            }
        }
        return -1;
    }

    private BallColor detectColor(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        // Lower threshold for dimmer balls
        if (r + g + b < 20) return BallColor.NONE;

        Color.RGBToHSV(r, g, b, hsv);
        float h = hsv[0];

        // GREEN
        if (h >= 25 && h <= 150) return BallColor.GREEN;

        // PURPLE (expanded range for better detection on first kicker)
        if ((h >= 160 && h <= 270) || (h >= 290 && h <= 360)) return BallColor.PURPLE;

        return BallColor.NONE;
    }
}
