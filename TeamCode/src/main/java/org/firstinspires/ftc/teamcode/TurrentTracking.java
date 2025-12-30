package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "TurrentTracking", group = "Test")
public class TurrentTracking extends LinearOpMode {

    // ====== DRIVE ======
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // ====== TURRET ======
    private DcMotorEx turret;

    // ====== OUTTAKE (shooter) ======
    private DcMotorEx outtake;

    // ====== KICKERS ======
    private Servo kick1;
    private Servo kick2;
    private Servo kick3;

    private static final double KICK_REST_POS = 0.0;
    private static final double KICK_FIRE_POS = 1.0;

    // ====== IMU ======
    private IMU imu;

    // encoder → degrees for turret
    private static final double TICKS_PER_DEGREE = 4.44;
    private static final double TURRET_SIGN = 1.0;   // flip to -1.0 if turret angle inverted
    private static final double HEADING_SIGN = -1.0; // flip to 1.0 if heading feels backwards

    private double headingFiltered = 0.0;
    private static final double HEADING_FILTER_ALPHA = 0.25;

    // world lock: field direction turret should face when tag is lost
    private double worldTargetAngle = 0.0;
    private boolean hasWorldTarget = false;

    // world-lock gains (softer to avoid shaking)
    private static final double K_AIM_WORLD = 0.03;
    private static final double MAX_TURRET_POWER_WORLD = 0.6;
    private static final double ANGLE_DEADBAND_DEG = 1.5;

    // ====== LIMELIGHT ======
    private Limelight3A limelight;

    private static final int TARGET_TAG_ID = 24;

    // Softer LL PID to reduce shaking
    private static final double LIMELIGHT_KP_TURN = 0.02;
    private static final double LIMELIGHT_MAX_TURN = 0.5;
    private static final double LIMELIGHT_AIM_TOLERANCE = 1.0;

    // offset for camera vs shooter. Leave 0 while we debug.
    private static final double AIM_OFFSET_DEG = 0.0;

    private double lastTx = 0.0;
    private double txFiltered = 0.0;          // low-pass filtered tx
    private static final double TX_FILTER_ALPHA = 0.3;

    private boolean haveTargetCached = false;
    private int lostTargetFrames = 0;
    private static final int MAX_LOST_FRAMES = 10;

    // ====== SHOOTER DISTANCE → RPM ======
    private static final double TICKS_PER_REV = 28.0;   // goBILDA 435rpm motor
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;

    // tune these based on what actually scores
    private static final double FAR_RPM   = 3500.0;  // when tag is small / far
    private static final double CLOSE_RPM = 2500.0;  // when tag is big / close

    // rough guess for Limelight targetArea (%) range, you will tune this
    private static final double AREA_FAR   = 1.0;    // small area = far
    private static final double AREA_CLOSE = 10.0;   // big area = close

    private double lastTargetArea = 0.0;
    private double lastComputedRpm = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== DRIVE =====
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== TURRET =====
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD); // flip if backwards
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== OUTTAKE =====
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE); // flip if wrong way
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== KICKERS =====
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");

        kick1.setPosition(KICK_REST_POS);
        kick2.setPosition(KICK_REST_POS);
        kick3.setPosition(KICK_REST_POS);

        // ===== IMU =====
        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
        headingFiltered = getSignedHeading();

        // ===== LIMELIGHT =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        telemetry.addLine("TurrentTracking Ready");
        telemetry.addLine("LT = auto aim (LL if tag, IMU world lock if no tag)");
        telemetry.addLine("RT = shooter (velocity based on tag distance)");
        telemetry.addLine("LB/RB = manual turret, Y/B/X = servos");
        telemetry.update();

        waitForStart();

        // Initialize world lock from starting direction (optional)
        double heading = headingFiltered;
        double turretDeg = getTurretDeg();
        worldTargetAngle = wrapAngle(heading + turretDeg);
        hasWorldTarget = true;

        while (opModeIsActive()) {

            // === IMU filtered heading ===
            double rawHeading = getSignedHeading();
            double dHead = wrapAngle(rawHeading - headingFiltered);
            headingFiltered = wrapAngle(headingFiltered + HEADING_FILTER_ALPHA * dHead);
            heading = headingFiltered;

            // turret angle
            turretDeg = getTurretDeg();

            // === simple drive ===
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;
            driveRobot(drive, strafe, turn);

            // servos
            kick1.setPosition(gamepad1.y ? KICK_FIRE_POS : KICK_REST_POS);
            kick2.setPosition(gamepad1.b ? KICK_FIRE_POS : KICK_REST_POS);
            kick3.setPosition(gamepad1.x ? KICK_FIRE_POS : KICK_REST_POS);

            // === Limelight: only ID 24 ===
            LLResult result = limelight.getLatestResult();
            boolean hasNow = false;
            double txNow = lastTx;
            double areaNow = lastTargetArea;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                if (fids != null && !fids.isEmpty()) {
                    for (LLResultTypes.FiducialResult fid : fids) {
                        if (fid.getFiducialId() == TARGET_TAG_ID) {
                            txNow = fid.getTargetXDegrees();
                            areaNow = fid.getTargetArea();
                            hasNow = true;
                            break;
                        }
                    }
                }
            }

            if (hasNow) {
                lastTx = txNow;
                lastTargetArea = areaNow;
                haveTargetCached = true;
                lostTargetFrames = 0;
            } else if (haveTargetCached && lostTargetFrames < MAX_LOST_FRAMES) {
                lostTargetFrames++;
            } else {
                haveTargetCached = false;
            }

            // smooth tx to reduce jitter (only when we see the tag)
            if (hasNow) {
                double txErr = txNow - txFiltered;
                txFiltered += TX_FILTER_ALPHA * txErr;
            }

            // === turret control ===
            boolean autoAim = gamepad1.left_trigger > 0.2;
            double turretPower = 0.0;
            String mode = "Idle";

            if (autoAim) {
                if (hasNow) {
                    // ==== LIMELIGHT DIRECT TRACKING (SMOOTHED) ====
                    double error = txFiltered + AIM_OFFSET_DEG;

                    if (Math.abs(error) < LIMELIGHT_AIM_TOLERANCE) {
                        turretPower = 0.0;
                    } else {
                        turretPower = LIMELIGHT_KP_TURN * error;
                        if (turretPower > LIMELIGHT_MAX_TURN) turretPower = LIMELIGHT_MAX_TURN;
                        if (turretPower < -LIMELIGHT_MAX_TURN) turretPower = -LIMELIGHT_MAX_TURN;
                    }

                    // when close to centered, capture worldTargetAngle for IMU fallback
                    if (Math.abs(error) < 2.0) {
                        worldTargetAngle = wrapAngle(heading + turretDeg);
                        hasWorldTarget = true;
                    }

                    mode = "LL tracking ID24";

                } else if (hasWorldTarget) {
                    // ==== IMU WORLD LOCK (less aggressive) ====
                    double desiredTurretDeg = wrapAngle(worldTargetAngle - heading);
                    double errorWorld = wrapAngle(desiredTurretDeg - turretDeg);

                    if (Math.abs(errorWorld) < ANGLE_DEADBAND_DEG) {
                        turretPower = 0.0;
                    } else {
                        turretPower = K_AIM_WORLD * errorWorld;
                        if (turretPower > MAX_TURRET_POWER_WORLD) turretPower = MAX_TURRET_POWER_WORLD;
                        if (turretPower < -MAX_TURRET_POWER_WORLD) turretPower = -MAX_TURRET_POWER_WORLD;
                    }

                    mode = "IMU world lock";
                } else {
                    turretPower = 0.0;
                    mode = "Auto, no lock";
                }
            } else {
                // manual turret
                double bump = 0.4;
                if (gamepad1.left_bumper) {
                    turretPower = -bump;
                } else if (gamepad1.right_bumper) {
                    turretPower = bump;
                } else {
                    turretPower = 0.0;
                }
                mode = "Manual";
            }

            turret.setPower(turretPower);

            double worldNow = wrapAngle(heading + turretDeg);

            // === SHOOTER VELOCITY FROM TAG "DISTANCE" ===
            boolean shoot = gamepad1.right_trigger > 0.2;
            if (shoot) {
                double rpm;
                if (haveTargetCached) {
                    rpm = computeRpmFromArea(lastTargetArea);
                } else {
                    // fallback when no tag cached – tune this
                    rpm = 3000.0;
                }
                rpm = clamp(rpm, MIN_RPM, MAX_RPM);
                lastComputedRpm = rpm;

                double ticksPerSecond = rpmToTicksPerSecond(rpm, TICKS_PER_REV);
                outtake.setVelocity(ticksPerSecond);
            } else {
                outtake.setVelocity(0.0);
            }

            telemetry.addData("Mode", mode);
            telemetry.addData("HeadingFilt", heading);
            telemetry.addData("TurretDeg", turretDeg);
            telemetry.addData("WorldNow", worldNow);
            telemetry.addData("WorldTarget", worldTargetAngle);
            telemetry.addData("HasWorldTarget", hasWorldTarget);
            telemetry.addData("HasNow(ID24)", hasNow);
            telemetry.addData("TagCached", haveTargetCached);
            telemetry.addData("LostFrames", lostTargetFrames);
            telemetry.addData("lastTx_raw", lastTx);
            telemetry.addData("txFiltered", txFiltered);
            telemetry.addData("TargetArea", lastTargetArea);
            telemetry.addData("ShooterRPM", lastComputedRpm);
            telemetry.addData("TurretPower", turretPower);
            telemetry.update();
        }

        driveRobot(0, 0, 0);
        turret.setPower(0.0);
        outtake.setVelocity(0.0);
    }

    // ===== helpers =====

    private double getSignedHeading() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return HEADING_SIGN * wrapAngle(yaw);
    }

    private double getTurretDeg() {
        return (turret.getCurrentPosition() / TICKS_PER_DEGREE) * TURRET_SIGN;
    }

    private double wrapAngle(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    private void driveRobot(double drive, double strafe, double turn) {
        double lf = drive + strafe + turn;
        double rf = drive - strafe - turn;
        double lb = drive - strafe + turn;
        double rb = drive + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double rpmToTicksPerSecond(double rpm, double tpr) {
        return rpm * tpr / 60.0;
    }

    private double computeRpmFromArea(double area) {
        // clamp area into [AREA_FAR, AREA_CLOSE]
        double a = clamp(area, AREA_FAR, AREA_CLOSE);

        // t = 0 → far, t = 1 → close
        double t = (a - AREA_FAR) / (AREA_CLOSE - AREA_FAR);

        // linear interpolate between FAR_RPM and CLOSE_RPM
        return FAR_RPM + t * (CLOSE_RPM - FAR_RPM);
    }
}
