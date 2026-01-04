package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "April Tag Test")
public class aprilTagTest extends LinearOpMode {

    // ================= CONSTANTS =================
    private static final double KICK_FIRE = 0.75;
    private static final double REST_1 = 0.125;
    private static final double REST_23 = 0.08;
    private static final double OUTTAKE_VELOCITY = 900;
    private static final long KICK_DELAY_MS = 1000;

    // ================= HARDWARE =================
    private Servo[] kickers = new Servo[3];
    private ColorSensor[] sensors = new ColorSensor[6];
    private DcMotorEx outtake;
    private Limelight3A limelight;

    private final float[] hsv = new float[3];

    enum BallColor { GREEN, PURPLE, NONE }

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

    private BallColor[] activePattern = null;
    private boolean patternLocked = false;

    private int patternIndex = 0;
    private long lastKickTime = 0;

    // ================= OPMODE =================
    @Override
    public void runOpMode() {

        // ---- Kickers ----
        kickers[0] = hardwareMap.get(Servo.class, "kick1");
        kickers[1] = hardwareMap.get(Servo.class, "kick2");
        kickers[2] = hardwareMap.get(Servo.class, "kick3");

        kickers[0].setDirection(Servo.Direction.FORWARD);
        kickers[1].setDirection(Servo.Direction.REVERSE);
        kickers[2].setDirection(Servo.Direction.REVERSE);

        // ---- Color Sensors ----
        sensors[0] = hardwareMap.get(ColorSensor.class, "color1");
        sensors[1] = hardwareMap.get(ColorSensor.class, "color2");

        sensors[2] = hardwareMap.get(ColorSensor.class, "color4");
        sensors[3] = hardwareMap.get(ColorSensor.class, "color3");

        sensors[4] = hardwareMap.get(ColorSensor.class, "color6");
        sensors[5] = hardwareMap.get(ColorSensor.class, "color5");

        // ---- Outtake ----
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setVelocity(OUTTAKE_VELOCITY); // always on

        // ---- Limelight ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // ---- Rest positions ----
        kickers[0].setPosition(REST_1);
        kickers[1].setPosition(REST_23);
        kickers[2].setPosition(REST_23);

        waitForStart();

        while (opModeIsActive()) {

            // ================= APRILTAG PATTERN SELECT =================
            if (!patternLocked) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                    if (fids != null && !fids.isEmpty()) {
                        int id = fids.get(0).getFiducialId();
                        if (id == 21) activePattern = PATTERN_GPP;
                        else if (id == 22) activePattern = PATTERN_PGP;
                        else if (id == 23) activePattern = PATTERN_PPG;

                        if (activePattern != null) {
                            patternLocked = true;
                            patternIndex = 0;
                        }
                    }
                }
            }

            // ================= SHOOTING =================
            if (gamepad1.x && patternLocked && patternIndex < 3) {

                if (System.currentTimeMillis() - lastKickTime > KICK_DELAY_MS) {

                    BallColor needed = activePattern[patternIndex];

                    BallColor s1 = detectColor(sensors[patternIndex * 2]);
                    BallColor s2 = detectColor(sensors[patternIndex * 2 + 1]);

                    if (s1 == needed || s2 == needed) {
                        kickers[patternIndex].setPosition(KICK_FIRE);
                        sleep(200);
                        kickers[patternIndex].setPosition(
                                patternIndex == 0 ? REST_1 : REST_23
                        );

                        patternIndex++;
                        lastKickTime = System.currentTimeMillis();
                    }
                }
            }

            // ================= TELEMETRY =================
            telemetry.addData("Pattern Locked", patternLocked);
            telemetry.addData("Pattern Index", patternIndex);

            telemetry.addData(
                    "Detected Colors",
                    detectColor(sensors[0]) + "," + detectColor(sensors[1]) + " | " +
                            detectColor(sensors[2]) + "," + detectColor(sensors[3]) + " | " +
                            detectColor(sensors[4]) + "," + detectColor(sensors[5])
            );

            telemetry.addData("Outtake", "ON");
            telemetry.update();
        }
    }

    // ================= COLOR DETECTION =================
    private BallColor detectColor(ColorSensor sensor) {
        Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsv);

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (sat < 0.25 || val < 0.15) return BallColor.NONE;

        if (hue >= 95 && hue <= 145) return BallColor.GREEN;
        if (hue >= 240 && hue <= 340) return BallColor.PURPLE;

        return BallColor.NONE;
    }
}
