package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;

@TeleOp(name = "TurretTrackingBlue", group = "Production")
public class TurrentTrackingBlue extends LinearOpMode {

    // -----------------------------------------------------------------------
    //  SHOOTING CONSTANTS
    // -----------------------------------------------------------------------
    private final int TARGET_ID = 20;
    private static final double HORIZONTAL_OFFSET = 0.0;

    // Fixed velocity presets (RPM)
    private static final double VELO_START    = 2600;
    private static final double VELO_BACK     = 2250;
    private static final double TICKS_PER_REV = 28.0;

    // -----------------------------------------------------------------------
    //  TURRET PID
    // -----------------------------------------------------------------------
    private double kP_Turret = 0.015;
    private double kD_Turret = 0.020;
    private static final double TURRET_DEADBAND_DEG = 2.5;

    private double previousError = 0;
    private double smoothedTx    = 0;
    private double filterWeight  = 0.12;

    // -----------------------------------------------------------------------
    //  TURRET CALIBRATION
    //  From ProTurretTracking: 384.5 ticks/motor rev * 4.075 gear ratio
    // -----------------------------------------------------------------------
    private static final double TICKS_PER_MOTOR_REV  = 384.5;
    private static final double CALIBRATED_RATIO      = 4.075;
    private static final double TICKS_PER_TURRET_REV  = TICKS_PER_MOTOR_REV * CALIBRATED_RATIO;
    private static final double SAFE_LIMIT            = 175.0;

    // -----------------------------------------------------------------------
    //  HARDWARE
    // -----------------------------------------------------------------------
    private DcMotorEx leftFront, rightFront, leftBack, rightBack, turret;
    private DcMotorEx outtakeL, outtakeR;
    private DcMotor   intake;
    private Servo[]   kickers = new Servo[4];
    private Limelight3A limelight;
    private final ColorSensor[] sensors = new ColorSensor[7];
    private RevBlinkinLedDriver led;
    private Servo Lift, hood;
    private IMU   imu;

    private static final double[] REST = {0.10, 0.00, 1.00, 1.00};
    private static final double[] FIRE = {0.70, 0.95, 0.45, 0.40};

    // -----------------------------------------------------------------------
    //  STATE
    // -----------------------------------------------------------------------
    private final boolean[] firing               = new boolean[4];
    private final boolean[] lastServoStateFiring  = new boolean[4];
    private final long[]    fireEndMs            = new long[4];

    private boolean prevDpadLeft = false, prevA2 = false, prevY2 = false, prevA1 = false;
    private boolean prevLB2 = false, prevRB2 = false;
    private boolean slowMode = false, intakeCancelStop = false;

    private int       sensorCycleIndex = 0;
    private double    lastSentVelocity = -1;
    private BallColor[] cachedColors   = new BallColor[7];
    private double    manualTargetRPM  = VELO_START;
    private boolean   targetVisible    = false;
    private long      jamClearTimer    = 0;
    private int       prevOccupied     = 0;

    // -----------------------------------------------------------------------
    //  TURRET AIM STATE  (IMU-based, from ProTurretTracking)
    // -----------------------------------------------------------------------
    private double  lastTargetWorldHeading = 0;
    private boolean hadTargetBefore        = false;

    // Cached once per loop — avoids calling IMU multiple times per iteration
    // which can return stale / inconsistent readings
    private double robotHeadingDeg = 0;

    // -----------------------------------------------------------------------
    //  MAIN LOOP
    // -----------------------------------------------------------------------
    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        initHardware();
        for (int i = 0; i < 7; i++) cachedColors[i] = BallColor.NONE;

        limelight.start();
        waitForStart();

        hood.setPosition(0.15);
        imu.resetYaw();

        // Initialise world heading to current robot heading so turret holds still on start
        lastTargetWorldHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            long now = System.currentTimeMillis();

            // Read IMU exactly once per loop — negated to match turret direction
            robotHeadingDeg = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            cachedColors[sensorCycleIndex] = detectColor(sensors[sensorCycleIndex], sensorCycleIndex);
            sensorCycleIndex = (sensorCycleIndex + 1) % 7;

            boolean[] hasBall = new boolean[4];
            hasBall[0] = isScorable(cachedColors[0]) || isScorable(cachedColors[1]);
            hasBall[1] = isScorable(cachedColors[2]) || isScorable(cachedColors[3]);
            hasBall[2] = isScorable(cachedColors[4]) || isScorable(cachedColors[5]);
            hasBall[3] = isScorable(cachedColors[6]);

            int occupied = 0;
            for (boolean b : hasBall) if (b) occupied++;
            boolean systemFull = (occupied >= 3);

            if (occupied >= 3 && prevOccupied < 3) jamClearTimer = now + 1000;
            prevOccupied = occupied;

            handleIntake(systemFull, now);

            if (gamepad2.start) manualTargetRPM = VELO_START;
            if (gamepad2.back)  manualTargetRPM = VELO_BACK;

            if (gamepad1.b) Lift.setPosition(0.5);
            else            Lift.setPosition(1.0);

            updateOuttake();
            updateLEDs(lastSentVelocity);
            processVision();
            handleTurret();

            if (gamepad1.a && !prevA1) slowMode = !slowMode;
            double driveScale = slowMode ? 0.4 : 1.0;
            driveRobot(-gamepad1.left_stick_y * driveScale,
                    gamepad1.left_stick_x * driveScale,
                    gamepad1.right_stick_x * driveScale);

            handleKickerInputs(now);
            updateKickerHardware(now);

            // --- TELEMETRY ---
            double turretDeg = (turret.getCurrentPosition() / TICKS_PER_TURRET_REV) * 360.0;

            telemetry.addData("[ TURRET ]",          "");
            telemetry.addData("  Target Visible",     targetVisible);
            telemetry.addData("  Smoothed Tx",        String.format("%.2f", smoothedTx));
            telemetry.addData("  World Aim (deg)",    String.format("%.2f", lastTargetWorldHeading));
            telemetry.addData("  Turret Angle (deg)", String.format("%.2f", (turret.getCurrentPosition() / TICKS_PER_TURRET_REV) * 360.0));
            telemetry.addData("[ IMU ]",              "");
            telemetry.addData("  Robot Heading",      String.format("%.2f°", robotHeadingDeg));
            telemetry.addData("  IMU OK?",            (robotHeadingDeg != 0.0 || hadTargetBefore) ? "YES" : "CHECK ORIENTATION");
            telemetry.addData("[ SHOOTER ]",          "");
            telemetry.addData("  RPM Target",         manualTargetRPM);
            telemetry.addData("  Occupied",           occupied);
            telemetry.update();

            prevA1       = gamepad1.a;
            prevA2       = gamepad2.a;
            prevY2       = gamepad2.y;
            prevLB2      = gamepad2.left_bumper;
            prevRB2      = gamepad2.right_bumper;
            prevDpadLeft = gamepad2.dpad_left;
        }
    }

    // -----------------------------------------------------------------------
    //  VISION
    // -----------------------------------------------------------------------
    private void processVision() {
        LLResult result = limelight.getLatestResult();
        targetVisible = false;
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                if (f.getFiducialId() == TARGET_ID) {
                    targetVisible = true;
                    double rawTx = f.getTargetXDegrees() + HORIZONTAL_OFFSET;
                    smoothedTx = (rawTx * filterWeight) + (smoothedTx * (1.0 - filterWeight));
                    break;
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    //  TURRET
    //
    //  Hold LEFT TRIGGER (Gamepad 1) to track.
    //  Release for manual: Left/Right Bumper.
    // -----------------------------------------------------------------------
    private void handleTurret() {
        if (gamepad1.left_trigger > 0.2) {

            double turretDeg    = (turret.getCurrentPosition() / TICKS_PER_TURRET_REV) * 360.0;
            double worldHeading = robotHeadingDeg + turretDeg;

            double error;

            if (targetVisible) {
                error = smoothedTx;

                // Save world heading when locked on
                if (Math.abs(error) < TURRET_DEADBAND_DEG) {
                    lastTargetWorldHeading = worldHeading;
                }
                hadTargetBefore = true;

            } else if (hadTargetBefore) {
                // IMU fallback: shortest path to saved world heading
                double targetRelative = normalizeAngle(lastTargetWorldHeading - robotHeadingDeg);
                error = normalizeAngle(targetRelative - turretDeg);

                // Anti-wrap: if shortest path crosses the physical limit, go the other way
                if (Math.abs(turretDeg + error) > SAFE_LIMIT) {
                    if (error > 0) error -= 360;
                    else           error += 360;
                }

            } else {
                turret.setPower(0);
                return;
            }

            // Hard safety clamps at physical limits
            if (turretDeg >=  SAFE_LIMIT && error > 0) { turret.setPower(0); previousError = error; return; }
            if (turretDeg <= -SAFE_LIMIT && error < 0) { turret.setPower(0); previousError = error; return; }

            // Deadband
            if (Math.abs(error) < TURRET_DEADBAND_DEG) {
                turret.setPower(0);
                previousError = error;
                return;
            }

            double derivative = error - previousError;
            double power      = (error * kP_Turret) + (derivative * kD_Turret);
            turret.setPower(clamp(power, -0.8, 0.8));
            previousError = error;

        } else {
            if (gamepad1.left_bumper)       turret.setPower(-0.5);
            else if (gamepad1.right_bumper) turret.setPower(0.5);
            else                            turret.setPower(0);
            previousError = 0;
        }
    }

    // -----------------------------------------------------------------------
    //  OUTTAKE
    // -----------------------------------------------------------------------
    private void updateOuttake() {
        double targetTicksPerSec = (manualTargetRPM * TICKS_PER_REV) / 60.0;
        double currentRequest = (gamepad2.right_trigger > 0.3) ? 0 : targetTicksPerSec;
        if (Math.abs(currentRequest - lastSentVelocity) > 5) {
            outtakeL.setVelocity(currentRequest);
            outtakeR.setVelocity(currentRequest);
            lastSentVelocity = currentRequest;
        }
    }

    // -----------------------------------------------------------------------
    //  HARDWARE INIT
    // -----------------------------------------------------------------------
    private void initHardware() {
        led        = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");
        turret     = hardwareMap.get(DcMotorEx.class, "turret");
        outtakeL   = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR   = hardwareMap.get(DcMotorEx.class, "outtakeR");
        intake     = hardwareMap.get(DcMotor.class,   "intake");
        Lift       = hardwareMap.get(Servo.class, "Lift");
        hood       = hardwareMap.get(Servo.class, "hood");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset turret encoder on init so degrees are relative to start position
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (int i = 0; i < 4; i++) kickers[i] = hardwareMap.get(Servo.class, "kick" + (i + 1));
        for (int i = 0; i < 7; i++) sensors[i]  = hardwareMap.get(ColorSensor.class, "color" + (i + 1));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    // -----------------------------------------------------------------------
    //  DRIVE
    // -----------------------------------------------------------------------
    private void driveRobot(double y, double x, double rx) {
        double lf = y + x + rx, rf = y - x - rx, lb = y - x + rx, rb = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
        leftFront.setPower(lf / max);  rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);   rightBack.setPower(rb / max);
    }

    // -----------------------------------------------------------------------
    //  INTAKE
    // -----------------------------------------------------------------------
    private void handleIntake(boolean systemFull, long now) {
        if (gamepad2.dpad_left && !prevDpadLeft) intakeCancelStop = true;
        if (!gamepad2.dpad_up) intakeCancelStop = false;
        if (now < jamClearTimer)     intake.setPower(-1.0);
        else if (gamepad2.dpad_down) intake.setPower(-1.0);
        else if (gamepad2.dpad_up)   intake.setPower((systemFull && !intakeCancelStop) ? 0 : 1.0);
        else                         intake.setPower(0);
    }

    // -----------------------------------------------------------------------
    //  KICKERS
    // -----------------------------------------------------------------------
    private void handleKickerInputs(long now) {
        if (gamepad2.a && !prevA2)   requestKick(0, now);
        if (gamepad2.b)              requestKick(1, now);
        if (gamepad2.x)              requestKick(2, now);
        if (gamepad2.y && !prevY2)   requestKick(3, now);
        if (gamepad2.left_bumper  && !prevLB2) { int p = findKickerByColor(BallColor.PURPLE); if (p != -1) requestKick(p, now); }
        if (gamepad2.right_bumper && !prevRB2) { int g = findKickerByColor(BallColor.GREEN);  if (g != -1) requestKick(g, now); }
    }

    private void updateKickerHardware(long now) {
        for (int i = 0; i < 4; i++) {
            if (firing[i]) {
                if (!lastServoStateFiring[i]) { kickers[i].setPosition(FIRE[i]); lastServoStateFiring[i] = true; }
                if (now > fireEndMs[i]) firing[i] = false;
            } else {
                if (lastServoStateFiring[i]) { kickers[i].setPosition(REST[i]); lastServoStateFiring[i] = false; }
            }
        }
    }

    private void requestKick(int id, long now) {
        if (id >= 0 && id < 4 && !firing[id]) { firing[id] = true; fireEndMs[id] = now + 350; }
    }

    private int findKickerByColor(BallColor target) {
        if (cachedColors[0] == target || cachedColors[1] == target) return 0;
        if (cachedColors[2] == target || cachedColors[3] == target) return 1;
        if (cachedColors[4] == target || cachedColors[5] == target) return 2;
        if (cachedColors[6] == target) return 3;
        return -1;
    }

    // -----------------------------------------------------------------------
    //  LEDs
    // -----------------------------------------------------------------------
    private void updateLEDs(double currentVelTicks) {
        double targetTicks = (manualTargetRPM * TICKS_PER_REV) / 60.0;
        if (currentVelTicks <= 0) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else {
            if (Math.abs(outtakeL.getVelocity() - targetTicks) < 50)
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
            else
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
        }
    }

    // -----------------------------------------------------------------------
    //  COLOR SENSOR
    // -----------------------------------------------------------------------
    private BallColor detectColor(ColorSensor s, int index) {
        if (s.alpha() < 120) return BallColor.NONE;
        float[] hsv = new float[3];
        Color.RGBToHSV(s.red(), s.green(), s.blue(), hsv);
        if (hsv[0] >= 200 && hsv[0] <= 350 && hsv[1] > 0.08) return BallColor.PURPLE;
        if (hsv[0] >= 60  && hsv[0] <= 170 && hsv[1] > (index == 6 ? 0.42 : 0.25)) return BallColor.GREEN;
        if (hsv[1] < 0.25) return BallColor.WHITE;
        return BallColor.NONE;
    }

    private boolean isScorable(BallColor color) {
        return color == BallColor.PURPLE || color == BallColor.GREEN;
    }

    // -----------------------------------------------------------------------
    //  UTILITIES
    // -----------------------------------------------------------------------
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double normalizeAngle(double angle) {
        while (angle >  180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    enum BallColor { GREEN, PURPLE, WHITE, NONE }
}