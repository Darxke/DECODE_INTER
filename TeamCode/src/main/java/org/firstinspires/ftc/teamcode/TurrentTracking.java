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
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

@TeleOp(name = "TurrentTrackingRed", group = "Production")
public class TurrentTracking extends LinearOpMode {

    // --- SHOOTING & PID CONSTANTS ---
    private final int TARGET_ID = 24;
    private static double HORIZONTAL_OFFSET = 5.6;
    private double START_RPM = 900, RPM_PER_INCH = 5;
    private double kP_Turret = 0.040, kD_Turret = 0.080;
    private double previousError = 0, smoothedTx = 0, filterWeight = 0.45;

    // --- HARDWARE ---
    private DcMotorEx leftFront, rightFront, leftBack, rightBack, turret;
    private DcMotorEx outtakeL, outtakeR;
    private DcMotor intake;
    private Servo[] kickers = new Servo[4];
    private Limelight3A limelight;
    private final ColorSensor[] sensors = new ColorSensor[7];
    private RevBlinkinLedDriver led;

    private static final double[] REST = {0.10, 0.00, 1.00, 1.00};
    private static final double[] FIRE = {0.70, 0.95, 0.45, 0.40};

    // --- STATE TRACKING ---
    private final boolean[] firing = new boolean[4];
    private final boolean[] lastServoStateFiring = new boolean[4];
    private final long[] fireEndMs = new long[4];
    private boolean prevDpadLeft = false, prevA2 = false, prevY2 = false, prevA1 = false;
    private boolean prevLB2 = false, prevRB2 = false;
    private boolean slowMode = false, intakeCancelStop = false;

    // --- OPTIMIZATION & TIMERS ---
    private int sensorCycleIndex = 0;
    private double lastSentVelocity = -1;
    private BallColor[] cachedColors = new BallColor[7];
    private double lastKnownTargetVelocity = 1200;
    private boolean id24Visible = false;
    private long jamClearTimer = 0;
    private int prevOccupied = 0;

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        initHardware();
        for (int i = 0; i < 7; i++) cachedColors[i] = BallColor.NONE;

        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            long now = System.currentTimeMillis();

            // 1. SENSOR CYCLE
            cachedColors[sensorCycleIndex] = detectColor(sensors[sensorCycleIndex], sensorCycleIndex);
            sensorCycleIndex = (sensorCycleIndex + 1) % 7;

            // 2. OCCUPANCY LOGIC (No White Stop)
            boolean[] hasBall = new boolean[4];
            hasBall[0] = (isScorable(cachedColors[0]) || isScorable(cachedColors[1]));
            hasBall[1] = (isScorable(cachedColors[2]) || isScorable(cachedColors[3]));
            hasBall[2] = (isScorable(cachedColors[4]) || isScorable(cachedColors[5]));
            hasBall[3] = (isScorable(cachedColors[6]));

            int occupied = 0;
            for(boolean b : hasBall) if(b) occupied++;
            boolean systemFull = (occupied >= 3);

            if (occupied >= 3 && prevOccupied < 3) jamClearTimer = now + 1000;
            prevOccupied = occupied;

            // 3. INTAKE & OUTTAKE
            handleIntake(systemFull, now);
            updateOuttake();
            updateLEDs(lastSentVelocity);

            // 4. VISION & TURRET
            processVision();
            handleTurret();

            // 5. DRIVE TRAIN (With Normalization Fix)
            if (gamepad1.a && !prevA1) slowMode = !slowMode;
            double driveScale = slowMode ? 0.4 : 1.0;
            driveRobot(-gamepad1.left_stick_y * driveScale,
                    gamepad1.left_stick_x * driveScale,
                    gamepad1.right_stick_x * driveScale);

            // 6. KICKERS & COLOR SENSING
            handleKickerInputs(now);
            updateKickerHardware(now);

            // Telemetry
            telemetry.addData("Drive Mode", slowMode ? "SLOW" : "NORMAL");
            telemetry.addData("Occupied Slots", occupied);
            telemetry.update();

            prevA1 = gamepad1.a; prevA2 = gamepad2.a; prevY2 = gamepad2.y;
            prevLB2 = gamepad2.left_bumper; prevRB2 = gamepad2.right_bumper;
            prevDpadLeft = gamepad2.dpad_left;
        }
    }

    private void driveRobot(double y, double x, double rx) {
        double lf = y + x + rx, rf = y - x - rx, lb = y - x + rx, rb = y + x - rx;
        // Normalization ensures the robot moves in the correct direction even at high speeds
        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
        leftFront.setPower(lf/max); rightFront.setPower(rf/max);
        leftBack.setPower(lb/max); rightBack.setPower(rb/max);
    }

    private void initHardware() {
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // --- BRAKE MODE SETTINGS ---
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

        for (int i = 0; i < 4; i++) kickers[i] = hardwareMap.get(Servo.class, "kick" + (i + 1));
        for (int i = 0; i < 7; i++) sensors[i] = hardwareMap.get(ColorSensor.class, "color" + (i + 1));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    // Helper methods (isScorable, handleIntake, updateOuttake, handleTurret, processVision, updateKickerHardware, updateLEDs, detectColor, findKickerByColor, requestKick, clamp) remain unchanged from V130.
    private boolean isScorable(BallColor color) { return (color == BallColor.PURPLE || color == BallColor.GREEN); }
    private void handleIntake(boolean systemFull, long now) {
        if (gamepad2.dpad_left && !prevDpadLeft) intakeCancelStop = true;
        if (!gamepad2.dpad_up) intakeCancelStop = false;
        if (now < jamClearTimer) intake.setPower(-1.0);
        else if (gamepad2.dpad_down) intake.setPower(-1.0);
        else if (gamepad2.dpad_up) intake.setPower((systemFull && !intakeCancelStop) ? 0 : 1.0);
        else intake.setPower(0);
    }
    private void updateOuttake() {
        double targetVel = (gamepad2.right_trigger > 0.3) ? 0 : lastKnownTargetVelocity;
        if (Math.abs(targetVel - lastSentVelocity) > 5) {
            outtakeL.setVelocity(targetVel); outtakeR.setVelocity(targetVel);
            lastSentVelocity = targetVel;
        }
    }
    private void handleTurret() {
        if (gamepad1.left_trigger > 0.2) {
            if (id24Visible) {
                double error = smoothedTx;
                double derivative = (error - previousError);
                double power = (error * kP_Turret) + (derivative * kD_Turret) + (Math.signum(error) * 0.16);
                turret.setPower(clamp(power, -0.8, 0.8));
                previousError = error;
            } else { turret.setPower(0); }
        } else {
            if (gamepad1.left_bumper) turret.setPower(-0.25);
            else if (gamepad1.right_bumper) turret.setPower(0.25);
            else turret.setPower(0);
            previousError = 0;
        }
    }
    private void handleKickerInputs(long now) {
        if (gamepad2.a && !prevA2) requestKick(0, now);
        if (gamepad2.b) requestKick(1, now);
        if (gamepad2.x) requestKick(2, now);
        if (gamepad2.y && !prevY2) requestKick(3, now);
        if (gamepad2.left_bumper && !prevLB2) { int pIdx = findKickerByColor(BallColor.PURPLE); if (pIdx != -1) requestKick(pIdx, now); }
        if (gamepad2.right_bumper && !prevRB2) { int gIdx = findKickerByColor(BallColor.GREEN); if (gIdx != -1) requestKick(gIdx, now); }
    }
    private void updateKickerHardware(long now) {
        for (int i = 0; i < 4; i++) {
            if (firing[i]) {
                if (!lastServoStateFiring[i]) { kickers[i].setPosition(FIRE[i]); lastServoStateFiring[i] = true; }
                if (now > fireEndMs[i]) firing[i] = false;
            } else { if (lastServoStateFiring[i]) { kickers[i].setPosition(REST[i]); lastServoStateFiring[i] = false; } }
        }
    }
    private void processVision() {
        LLResult result = limelight.getLatestResult(); id24Visible = false;
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                if (f.getFiducialId() == TARGET_ID) {
                    id24Visible = true;
                    double distInches = Math.abs(f.getTargetPoseRobotSpace().getPosition().z * 39.37);
                    lastKnownTargetVelocity = clamp(START_RPM + (distInches * RPM_PER_INCH), 1000, 2000);
                    double rawTx = f.getTargetXDegrees() + HORIZONTAL_OFFSET;
                    smoothedTx = (rawTx * filterWeight) + (smoothedTx * (1.0 - filterWeight));
                    break;
                }
            }
        }
    }
    private void updateLEDs(double targetVel) {
        if (targetVel <= 0) led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        else {
            if (Math.abs(outtakeL.getVelocity() - targetVel) < 50) led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
            else led.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
        }
    }
    private BallColor detectColor(ColorSensor s, int index) {
        if (s.alpha() < 120) return BallColor.NONE;
        float[] hsv = new float[3]; Color.RGBToHSV(s.red(), s.green(), s.blue(), hsv);
        if (hsv[0] >= 200 && hsv[0] <= 350 && hsv[1] > 0.08) return BallColor.PURPLE;
        if (hsv[0] >= 60 && hsv[0] <= 170 && hsv[1] > (index == 6 ? 0.42 : 0.25)) return BallColor.GREEN;
        if (hsv[1] < 0.25) return BallColor.WHITE;
        return BallColor.NONE;
    }
    private int findKickerByColor(BallColor target) {
        if ((cachedColors[0] == target || cachedColors[1] == target)) return 0;
        if ((cachedColors[2] == target || cachedColors[3] == target)) return 1;
        if (cachedColors[4] == target || cachedColors[5] == target) return 2;
        if (cachedColors[6] == target) return 3;
        return -1;
    }
    private void requestKick(int id, long now) { if (id >= 0 && id < 4 && !firing[id]) { firing[id] = true; fireEndMs[id] = now + 350; } }
    private double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }

    enum BallColor { GREEN, PURPLE, WHITE, NONE }
}