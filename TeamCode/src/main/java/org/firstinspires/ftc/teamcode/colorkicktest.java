package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Color Kick Test")
public class colorkicktest extends LinearOpMode {

    private static final double KICK_FIRE = 0.75;
    private static final double OUTTAKE_VELOCITY = 900;

    private Servo[] kickers = new Servo[3];
    private ColorSensor[] sensors = new ColorSensor[6];
    private DcMotorEx outtake;

    private final float[] hsv = new float[3];

    // track which kicker should stay up
    private boolean[] kickerActive = new boolean[3];

    enum BallColor { GREEN, PURPLE, NONE }

    @Override
    public void runOpMode() {

        // ---------------------------
        // HARDWARE MAP
        // ---------------------------
        kickers[0] = hardwareMap.get(Servo.class, "kick1"); // kicker 1
        kickers[1] = hardwareMap.get(Servo.class, "kick2"); // kicker 2 mapped to index 2
        kickers[2] = hardwareMap.get(Servo.class, "kick3"); // kicker 3 mapped to index 1

        // Servo directions
        kickers[0].setDirection(Servo.Direction.FORWARD);
        kickers[1].setDirection(Servo.Direction.REVERSE);
        kickers[2].setDirection(Servo.Direction.REVERSE);

        // Flip sensors for kicker2 & kicker3
        sensors[0] = hardwareMap.get(ColorSensor.class, "color1");
        sensors[1] = hardwareMap.get(ColorSensor.class, "color2");
        sensors[2] = hardwareMap.get(ColorSensor.class, "color4"); // kicker2 flipped
        sensors[3] = hardwareMap.get(ColorSensor.class, "color3"); // kicker2 flipped
        sensors[4] = hardwareMap.get(ColorSensor.class, "color6"); // kicker3 flipped
        sensors[5] = hardwareMap.get(ColorSensor.class, "color5"); // kicker3 flipped

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake.setDirection(DcMotor.Direction.REVERSE); // reversed
        outtake.setVelocity(OUTTAKE_VELOCITY); // always on

        // ---------------------------
        // INITIAL KICKER POSITIONS
        // ---------------------------
        for (int i = 0; i < 3; i++) {
            if (i == 2) kickers[i].setPosition(0.08); // kicker 2 resting at 0.08
            else kickers[i].setPosition(0.125); // others resting at 0.125
        }

        waitForStart();

        while (opModeIsActive()) {

            int kickerToFire = -1;
            BallColor targetColor = null;

            if (gamepad1.x) targetColor = BallColor.GREEN;
            else if (gamepad1.b) targetColor = BallColor.PURPLE;

            // ---------------------------
            // FIND FIRST KICKER THAT MATCHES TARGET COLOR
            // ---------------------------
            if (targetColor != null) {
                for (int i = 0; i < 3; i++) {
                    BallColor s1 = detectColor(sensors[i*2]);
                    BallColor s2 = detectColor(sensors[i*2 + 1]);

                    if ((s1 == targetColor || s2 == targetColor)) {
                        kickerToFire = i;
                        break; // only one kicker at a time
                    }
                }
            }

            // ---------------------------
            // UPDATE kickerActive ARRAY
            // ---------------------------
            for (int i = 0; i < 3; i++) {
                if (i == kickerToFire) kickerActive[i] = true;
                else if (!gamepad1.x && !gamepad1.b) kickerActive[i] = false; // reset only when no button pressed
            }

            // ---------------------------
            // ACTUATE KICKERS BASED ON kickerActive
            // ---------------------------
            for (int i = 0; i < 3; i++) {
                if (kickerActive[i]) kickers[i].setPosition(KICK_FIRE);
                else {
                    if (i == 2) kickers[i].setPosition(0.08);
                    else kickers[i].setPosition(0.125);
                }
            }

            // ---------------------------
            // TELEMETRY
            // ---------------------------
            telemetry.addData("Target Color", targetColor);
            telemetry.addData("Detected Colors",
                    detectColor(sensors[0]) + "," + detectColor(sensors[1]) + " | " +
                            detectColor(sensors[2]) + "," + detectColor(sensors[3]) + " | " +
                            detectColor(sensors[4]) + "," + detectColor(sensors[5]));
            telemetry.addData("Kicker Fired", kickerToFire + 1);
            telemetry.addData("Outtake Velocity", OUTTAKE_VELOCITY);
            telemetry.update();
        }
    }

    // ---------------------------
    // HSV COLOR DETECTION
    // ---------------------------
    private BallColor detectColor(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        Color.RGBToHSV(r, g, b, hsv);

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        // Relaxed thresholds for low light / purple detection
        if (r + g + b < 20) return colorkicktest.BallColor.NONE;
        // Reject red
        if ((hue >= 0 && hue <= 25) || (hue >= 330 && hue <= 360)) return BallColor.NONE;

        // Green threshold
        if (hue >= 25 && hue <= 150) return colorkicktest.BallColor.GREEN;

        // PURPLE (expanded range for better detection on first kicker)
        if ((hue >= 160 && hue <= 270) || (hue >= 290 && hue <= 360)) return colorkicktest.BallColor.PURPLE;

        return BallColor.NONE;
    }
}
