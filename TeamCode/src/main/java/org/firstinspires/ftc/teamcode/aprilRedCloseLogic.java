package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@Autonomous(name="aprilRedCloseLogic", group="Autonomous")
public class aprilRedCloseLogic extends LinearOpMode {

    // ===== HARDWARE =====
    private DcMotorEx turret;
    private DcMotorEx outtake;

    private DcMotorEx intake;
    private ColorSensor[] sensors = new ColorSensor[6];
    private Servo kick1, kick2, kick3;
    private Limelight3A limelight;
    private MecanumDrive drive;

    // ===== TURNTABLE POSITIONS (encoder ticks) =====
    private static final int LEFT_SCAN_TICKS = -350;  // turret turned left
    private static final int FORWARD_TICKS = 0;       // forward shooting

    // ===== KICKER POSITIONS =====
    private static final double[] KICK_FIRE = {0.7, 1, 0.45};
    private static final double[] KICK_REST = {0.1, 0, 1.0};

    // ===== SHOOT PATTERNS =====
    enum BallColor { GREEN, PURPLE, NONE }

    private static final BallColor[] PATTERN_PPG = { BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN };
    private static final BallColor[] PATTERN_PGP = { BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE };
    private static final BallColor[] PATTERN_GPP = { BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };

    private BallColor[] activePattern = null;

    // ===== SHOOTING STATE =====
    private boolean shooting = false;
    private int shootIndex = 0;
    private int shootState = 0;
    private long stateTime = 0;
    private int activeKicker = -1;
    private boolean[] kickerUsed = new boolean[3];

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== HARDWARE MAP =====
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(LEFT_SCAN_TICKS);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(0.5);

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        kickersInit();

        // Color sensors
        sensors[0] = hardwareMap.get(ColorSensor.class, "color1");
        sensors[1] = hardwareMap.get(ColorSensor.class, "color2");
        sensors[2] = hardwareMap.get(ColorSensor.class, "color3");
        sensors[3] = hardwareMap.get(ColorSensor.class, "color4");
        sensors[4] = hardwareMap.get(ColorSensor.class, "color5");
        sensors[5] = hardwareMap.get(ColorSensor.class, "color6");

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Roadrunner start pose
        Pose2d startPose = new Pose2d(-52, 52, Math.toRadians(-222));
        drive = new MecanumDrive(hardwareMap, startPose);

        // ===== INIT LOOP TELEMETRY: APRILTAG + PATTERN =====
        while (!isStarted() && !isStopRequested()) {
            activePattern = null;
            int seenId = -1;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                if (fids != null && !fids.isEmpty()) {
                    seenId = fids.get(0).getFiducialId();

                    if (seenId == 21) activePattern = PATTERN_GPP;
                    else if (seenId == 22) activePattern = PATTERN_PGP;
                    else if (seenId == 23) activePattern = PATTERN_PPG;
                }
            }

            telemetry.addLine("=== INIT APRILTAG CHECK ===");
            telemetry.addData("Tag Detected", seenId != -1);
            telemetry.addData("Tag ID", seenId == -1 ? "NONE" : seenId);
            telemetry.addData("Pattern", activePattern == null ? "NONE" : activePatternToString());
            telemetry.update();

            sleep(50);
        }

        // ===== START =====
        telemetry.addLine("AprilRedClose ready");
        telemetry.update();
        waitForStart();

        // ===== START INTAKE =====
        outtake.setVelocity(1250); // RPM
        if (isStopRequested()) return;

        // ===== DRIVE BACKWARD WHILE SCANNING =====
        Action moveBackward = drive.actionBuilder(startPose)
                .strafeToConstantHeading(new Vector2d(-20, 20))
                .build();
        Actions.runBlocking(moveBackward);

        // ===== TURRET GOES BACK TO SHOOTING POSITION ONCE APRILTAG DETECTED =====
        updatePatternFromLimelight(); // check one last time after moving
        if (activePattern != null) {
            turret.setTargetPosition(FORWARD_TICKS);
            turret.setPower(0.5);
            while (opModeIsActive() && turret.isBusy()) {
                telemetry.addData("Turret", turret.getCurrentPosition());
                telemetry.addData("Pattern", activePatternToString());
                telemetry.update();
            }
        }

        // ===== FIRST SHOOT =====
        shootSequence();
        intake.setPower(1);

        // ===== CYCLE 1 MOVEMENT =====
        Action cycleMove = drive.actionBuilder(new Pose2d(-20, 20, Math.toRadians(-222)))
                .strafeToLinearHeading(new Vector2d(-10, 35), Math.toRadians(-260))
                .waitSeconds(.1)
                .strafeToConstantHeading(new Vector2d(-12.5, 65.5))
                .waitSeconds(.25)
                .build();
        Actions.runBlocking(cycleMove);

        Action shoot2 = drive.actionBuilder(new Pose2d(-12.5,65.5, Math.toRadians(-260)))
                .strafeToLinearHeading(new Vector2d(-20,20), Math.toRadians(-222))
                .build();

        Action cycle2 = drive.actionBuilder(new Pose2d(-20,20, Math.toRadians(-222)))
                .strafeToLinearHeading(new Vector2d(11.5, 40), Math.toRadians(-260))
                .waitSeconds(.1)
                .strafeToConstantHeading(new Vector2d(10, 79))
                .waitSeconds(.5)
                .build();

        Action shoot3 = drive.actionBuilder(new Pose2d(10,79, Math.toRadians(-260)))
                .strafeToConstantHeading(new Vector2d(11,50))
                .waitSeconds(.1)
                .strafeToLinearHeading(new Vector2d(-20,20), Math.toRadians(-222))
                .build();
        Action park = drive.actionBuilder(new Pose2d(-20,20, Math.toRadians(-222)))
                .strafeToLinearHeading(new Vector2d(9.5,50), Math.toRadians(-260))
                .build();
        // ===== RETURN TURRET TO SHOOTING POSITION AFTER CYCLE =====
        turret.setTargetPosition(FORWARD_TICKS);
        turret.setPower(0.5);
        while (opModeIsActive() && turret.isBusy()) {
            telemetry.addData("Turret", turret.getCurrentPosition());
            telemetry.update();
        }

        // ===== SECOND SHOOT =====
        intake.setPower(-1);
        Actions.runBlocking(shoot2);
        intake.setPower(0);
        shootSequence();
        intake.setPower(1);
        Actions.runBlocking(cycle2);
        intake.setPower(-1);
        Actions.runBlocking(shoot3);
        intake.setPower(0);
        shootSequence();
        Actions.runBlocking(park);

        // ===== STOP INTAKE AT END =====
        outtake.setVelocity(0);
    }

    // ----------------------- HELPERS -----------------------

    private void kickersInit() {
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");
        kick1.setPosition(KICK_REST[0]);
        kick2.setPosition(KICK_REST[1]);
        kick3.setPosition(KICK_REST[2]);
    }

    private void updatePatternFromLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
            if (fids != null) {
                for (LLResultTypes.FiducialResult fid : fids) {
                    int id = fid.getFiducialId();
                    if (id == 21) activePattern = PATTERN_GPP;
                    if (id == 22) activePattern = PATTERN_PGP;
                    if (id == 23) activePattern = PATTERN_PPG;
                }
            }
        }
    }

    private void shootSequence() {
        if (activePattern == null) return;

        shooting = true;
        shootIndex = 0;
        shootState = 0;
        activeKicker = -1;
        stateTime = System.currentTimeMillis();
        for (int i = 0; i < 3; i++) kickerUsed[i] = false;

        while (opModeIsActive() && shooting) {
            long now = System.currentTimeMillis();

            if (shootIndex < 3) {
                switch (shootState) {
                    case 0: // find kicker + fire
                        activeKicker = findMatchingKicker(activePattern[shootIndex]);

                        // FALLBACK: fire whatever is loaded if no match
                        if (activeKicker == -1) {
                            activeKicker = findAnyLoadedKicker();
                            telemetry.addLine("Fallback firing: no color match");
                        }

                        if (activeKicker != -1) {
                            setKicker(activeKicker, KICK_FIRE[activeKicker]);
                            stateTime = now;
                            shootState = 1;
                        } else {
                            // Nothing loaded at all, skip this shot
                            shootIndex++;
                        }
                        break;

                    case 1: // wait up (longer)
                        if (now - stateTime >= 500) { // 500ms instead of 350ms
                            setKicker(activeKicker, KICK_REST[activeKicker]);
                            stateTime = now;
                            shootState = 2;
                        }
                        break;

                    case 2: // wait down + interkick
                        if (now - stateTime >= 650) {
                            shootIndex++;
                            shootState = 0;
                            activeKicker = -1;
                        }
                        break;
                }
            } else {
                shooting = false;
            }

            // ----- TELEMETRY -----
            telemetry.addData("Pattern", activePatternToString());
            telemetry.addData("K1", detectColor(sensors[0]) + "," + detectColor(sensors[1]));
            telemetry.addData("K2", detectColor(sensors[2]) + "," + detectColor(sensors[3]));
            telemetry.addData("K3", detectColor(sensors[4]) + "," + detectColor(sensors[5]));
            telemetry.addData("Shoot Index", shootIndex);
            telemetry.addData("Shoot State", shootState);
            telemetry.addData("Active Kicker", activeKicker);
            telemetry.update();
        }
    }

    private void setKicker(int i, double pos) {
        if (i == 0) kick1.setPosition(pos);
        if (i == 1) kick2.setPosition(pos);
        if (i == 2) kick3.setPosition(pos);
    }

    private int findMatchingKicker(BallColor target) {
        for (int i = 0; i < 3; i++) {
            if (kickerUsed[i]) continue;
            if (detectColor(sensors[i*2]) == target || detectColor(sensors[i*2+1]) == target) {
                kickerUsed[i] = true;
                return i;
            }
        }
        return -1;
    }

    // FALLBACK helper: find any kicker that has a ball (not NONE)
    private int findAnyLoadedKicker() {
        for (int i = 0; i < 3; i++) {
            if (kickerUsed[i]) continue;

            BallColor c1 = detectColor(sensors[i * 2]);
            BallColor c2 = detectColor(sensors[i * 2 + 1]);

            if (c1 != BallColor.NONE || c2 != BallColor.NONE) {
                kickerUsed[i] = true;
                return i;
            }
        }
        return -1;
    }

    private BallColor detectColor(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        if (r + g + b < 20) return BallColor.NONE;

        float[] hsvLocal = new float[3];
        android.graphics.Color.RGBToHSV(r, g, b, hsvLocal);
        float h = hsvLocal[0];

        if (h >= 30 && h <= 150) return BallColor.GREEN;
        if ((h >= 240 && h <= 360)) return BallColor.PURPLE;

        return BallColor.NONE;
    }

    private String activePatternToString() {
        if (activePattern == null) return "NONE";
        return activePattern[0] + " " + activePattern[1] + " " + activePattern[2];
    }
}