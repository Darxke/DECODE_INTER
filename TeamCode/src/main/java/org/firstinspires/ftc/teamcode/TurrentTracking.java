package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "Turrent_OG_Final_Fixed_Mapping", group = "Production")
public class TurrentTracking extends LinearOpMode {

    private DcMotorEx leftFront, rightFront, leftBack, rightBack, turret, outtake;
    private DcMotor intake;
    private Servo kick1, kick2, kick3;
    private RevBlinkinLedDriver led;
    private Limelight3A limelight;
    private final ColorSensor[] sensors = new ColorSensor[6];
    private List<LynxModule> allHubs;

    enum BallColor { GREEN, PURPLE, NONE }

    private static final double VELOCITY_TOLERANCE = 45.0;

    // ---------------- SHOOTER TUNING ----------------
    // If close is too strong, lower CLOSE and/or BASE_RPM.
    private static final double BASE_RPM = 1600;

    private static final double RPM_SCALE_CLOSE = 1.62;  // weaker close
    private static final double RPM_SCALE_FAR   = 1.72;  // stronger far

    // TY-based hysteresis thresholds (ABS(TY))
    // Enter FAR when abs(ty) goes ABOVE ENTER (assumption)
    // Exit FAR when abs(ty) goes BELOW EXIT  (assumption)
    // If your camera behaves opposite, flip the comparisons in getRpmScaleForDistance().
    private static final double TY_FAR_ENTER = 10.0;
    private static final double TY_FAR_EXIT  = 8.0;

    // Remembers last mode so losing the tag doesn't force "close"
    private boolean farMode = false;
    // ------------------------------------------------

    // Servo Positions
    private static final double K1_REST = 0.1, K1_FIRE = 0.7;
    private static final double K2_REST = 0.1, K2_FIRE = 0.95;
    private static final double K3_REST = 1.0, K3_FIRE = 0.45;

    @Override
    public void runOpMode() throws InterruptedException {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        DcMotorEx[] driveMotors = {leftFront, rightFront, leftBack, rightBack};
        for (DcMotorEx m : driveMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setDirection(m == leftFront || m == leftBack ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        }

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        for (int i = 0; i < 6; i++) sensors[i] = hardwareMap.get(ColorSensor.class, "color" + (i + 1));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        setAllKickersRest();
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            driveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // Intake control
            intake.setPower((gamepad1.a || gamepad2.y) ? 1.0 : ((gamepad1.b || gamepad2.a) ? -1.0 : 0));

            boolean aimEnabled = gamepad2.left_trigger > 0.2;
            boolean shootEnabled = gamepad2.right_trigger > 0.2;

            LLResult latest = limelight.getLatestResult();

            // Turret aim (TX)
            if (aimEnabled) {
                if (latest != null && latest.isValid()) turret.setPower(latest.getTx() * 0.04);
                else turret.setPower(0);
            } else {
                turret.setPower(gamepad2.right_bumper ? 0.5 : (gamepad2.left_bumper ? -0.5 : 0));
            }

            // Shooter
            double scale = getRpmScaleForDistance(latest);
            double targetVel = (BASE_RPM * scale * 28.0) / 60.0;

            if (shootEnabled) {
                outtake.setVelocity(targetVel);

                if (Math.abs(outtake.getVelocity() - targetVel) < VELOCITY_TOLERANCE) {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                } else {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
                }

                if (gamepad2.x) {
                    int k = findKickerForColor(BallColor.PURPLE);
                    if (k != -1) fireKickerOG(k);
                } else if (gamepad2.b) {
                    int k = findKickerForColor(BallColor.GREEN);
                    if (k != -1) fireKickerOG(k);
                }

            } else {
                outtake.setVelocity(0);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
        }
    }

    private double getRpmScaleForDistance(LLResult result) {
        if (result == null || !result.isValid()) {
            return farMode ? RPM_SCALE_FAR : RPM_SCALE_CLOSE;
        }

        double ty;
        try {
            ty = Math.abs(result.getTy());
        } catch (Exception e) {
            return farMode ? RPM_SCALE_FAR : RPM_SCALE_CLOSE;
        }


        if (!farMode && ty > TY_FAR_ENTER) farMode = true;
        else if (farMode && ty < TY_FAR_EXIT) farMode = false;

        return farMode ? RPM_SCALE_FAR : RPM_SCALE_CLOSE;
    }

    private void fireKickerOG(int idx) {
        // --- FIXED MAPPING ---
        if (idx == 0) {
            kick1.setPosition(K1_FIRE);
            sleep(350);
            kick1.setPosition(K1_REST);
        } else if (idx == 1) {
            kick2.setPosition(K2_FIRE);
            sleep(350);
            kick2.setPosition(K2_REST);
        } else if (idx == 2) {
            kick3.setPosition(K3_FIRE);
            sleep(350);
            kick3.setPosition(K3_REST);
        }
    }

    private BallColor getBallColor(ColorSensor s) {
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(s.red(), s.green(), s.blue(), hsv);
        if (hsv[0] >= 30 && hsv[0] <= 150) return BallColor.GREEN;
        if (hsv[0] >= 240) return BallColor.PURPLE;
        return BallColor.NONE;
    }

    private int findKickerForColor(BallColor target) {
        if (getBallColor(sensors[0]) == target || getBallColor(sensors[1]) == target) return 0;

        if (getBallColor(sensors[4]) == target || getBallColor(sensors[5]) == target) return 2;

        if (getBallColor(sensors[2]) == target || getBallColor(sensors[3]) == target) return 1;

        return -1;
    }

    private void driveRobot(double d, double s, double t) {
        double lf = d + s + t, rf = d - s - t, lb = d - s + t, rb = d + s - t;
        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));
        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
    }

    private void setAllKickersRest() {
        kick1.setPosition(K1_REST);
        kick2.setPosition(K2_REST);
        kick3.setPosition(K3_REST);
    }
}