package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "TurrentTracking", group = "Test")
public class TurrentTracking extends LinearOpMode {

    // ===== DRIVE =====
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // ===== TURRET =====
    private DcMotorEx turret;

    // ===== OUTTAKE =====
    private DcMotor outtake;

    // ===== SERVOS (Y, B, X) =====
    private Servo kick1;
    private Servo kick2;
    private Servo kick3;

    // ===== IMU =====
    private IMU imu;

    // ===== LIMELIGHT =====
    private Limelight3A limelight;
    private static final int TARGET_TAG_ID = 24;   // Only track this tag

    // ===== ENCODER → DEGREES =====
    private static final double TICKS_PER_DEGREE = 4.44;

    // Signs (flip if needed)
    private static final double TURRET_SIGN = 1.0;
    private static final double HEADING_SIGN = -1.0;

    // IMU world-angle PID (fallback & world lock)
    private static final double K_AIM_WORLD = 0.025;
    private static final double MAX_TURRET_POWER_WORLD = 0.5;
    private static final double ANGLE_DEADBAND_DEG = 2.5;

    // Limelight PID (direct tx tracking)
    private static final double K_AIM_LL = 0.015;
    private static final double MAX_TURRET_POWER_LL = 0.6;
    private static final double LL_DEADBAND_DEG = 1.0;

    // Smoothing
    private static final double TX_FILTER_ALPHA = 0.25;
    private static final double HEADING_FILTER_ALPHA = 0.2;

    // World direction turret should face
    private double worldTargetAngle = 0.0;
    private boolean hasWorldTarget = false;

    // Filtered values
    private double headingFiltered = 0.0;
    private double txFiltered = 0.0;

    private boolean lastA = false;

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
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== OUTTAKE (reverse so it spins opposite) =====
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setPower(0.6);

        // ===== SERVOS =====
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");

        // ===== IMU =====
        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
        headingFiltered = getSignedHeading();

        // ===== LIMELIGHT =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);         // make sure pipeline 0 is your AprilTag pipeline
        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addLine("TurrentTracking: LL+IMU hybrid, outtake + servos");
        telemetry.addLine("A = lock world dir, LT = auto track, LB/RB = manual turret");
        telemetry.update();

        waitForStart();

        double heading = headingFiltered;
        double turretDeg = getTurretDeg();
        worldTargetAngle = wrapAngle(heading + turretDeg);
        hasWorldTarget = true;
        txFiltered = 0.0;

        while (opModeIsActive()) {

            // ===== HEADING FILTER =====
            double rawHeading = getSignedHeading();
            double dHead = wrapAngle(rawHeading - headingFiltered);
            headingFiltered = wrapAngle(headingFiltered + HEADING_FILTER_ALPHA * dHead);
            heading = headingFiltered;

            // ===== TURRET ANGLE =====
            turretDeg = getTurretDeg();
            int turretTicks = turret.getCurrentPosition();

            // ===== DRIVE =====
            boolean slowMode = gamepad1.right_bumper;
            double speedScale = slowMode ? 0.5 : 1.0;

            double drive  = -gamepad1.left_stick_y * speedScale;
            double strafe =  gamepad1.left_stick_x * speedScale;
            double turn   =  gamepad1.right_stick_x * speedScale;

            double lf = drive + strafe + turn;
            double rf = drive - strafe - turn;
            double lb = drive - strafe + turn;
            double rb = drive + strafe - turn;

            leftFront.setPower(lf);
            rightFront.setPower(rf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);

            // ===== WORLD LOCK WITH A =====
            boolean aPressed = gamepad1.a;
            if (aPressed && !lastA) {
                turretDeg = getTurretDeg();
                worldTargetAngle = wrapAngle(heading + turretDeg);
                hasWorldTarget = true;
            }
            lastA = aPressed;

            // ===== SERVOS ON Y/B/X =====
            if (gamepad1.y) kick1.setPosition(1.0);
            if (gamepad1.b) kick2.setPosition(0);
            if (gamepad1.x) kick3.setPosition(0);

            // ===== LIMELIGHT: look for ID 24 =====
            double txRaw = 0.0;
            boolean hasTag24 = false;
            int lastSeenId = -1;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                if (fids != null && !fids.isEmpty()) {
                    for (LLResultTypes.FiducialResult fid : fids) {
                        int fidId = fid.getFiducialId();
                        double fidTx = fid.getTargetXDegrees();
                        lastSeenId = fidId;

                        telemetry.addData("SeenTagID", fidId);
                        telemetry.addData("TagTx", fidTx);

                        if (fidId == TARGET_TAG_ID) {
                            txRaw = fidTx;
                            hasTag24 = true;
                        }
                    }
                }
            }

            boolean autoTrack = gamepad1.left_trigger > 0.2;
            double turretPower = 0.0;
            String mode = "Idle";

            if (autoTrack) {
                if (hasTag24) {
                    // ===== DIRECT LL TRACKING ON ID 24 =====
                    txFiltered = txFiltered + TX_FILTER_ALPHA * (txRaw - txFiltered);

                    // If it moves away from the tag, flip the sign here:
                    double errorLL = -txFiltered;

                    if (Math.abs(errorLL) < LL_DEADBAND_DEG) {
                        turretPower = 0.0;
                    } else {
                        turretPower = K_AIM_LL * errorLL;
                        if (turretPower > MAX_TURRET_POWER_LL) turretPower = MAX_TURRET_POWER_LL;
                        if (turretPower < -MAX_TURRET_POWER_LL) turretPower = -MAX_TURRET_POWER_LL;
                    }

                    // Update world target based on where we are while on the tag
                    worldTargetAngle = wrapAngle(heading + turretDeg);
                    hasWorldTarget = true;

                    mode = "LL tracking ID24";

                } else if (hasWorldTarget) {
                    // ===== IMU WORLD LOCK FALLBACK =====
                    double desiredTurretDeg = wrapAngle(worldTargetAngle - heading);
                    double errorWorld = wrapAngle(desiredTurretDeg - turretDeg);

                    if (Math.abs(errorWorld) < ANGLE_DEADBAND_DEG) {
                        turretPower = 0.0;
                    } else {
                        turretPower = K_AIM_WORLD * errorWorld;
                        if (turretPower > MAX_TURRET_POWER_WORLD) turretPower = MAX_TURRET_POWER_WORLD;
                        if (turretPower < -MAX_TURRET_POWER_WORLD) turretPower = -MAX_TURRET_POWER_WORLD;
                    }

                    mode = "IMU world-lock fallback";
                } else {
                    turretPower = 0.0;
                    mode = "Auto, no tag, no lock";
                }

            } else {
                // ===== MANUAL TURRET (when LT not held) =====
                double bumpPower = 0.35;
                if (gamepad1.left_bumper) {
                    turretPower = -bumpPower;
                } else if (gamepad1.right_bumper) {
                    turretPower = bumpPower;
                } else {
                    turretPower = 0;
                }
                mode = "Manual";
            }

            turret.setPower(turretPower);

            double worldNow = wrapAngle(heading + turretDeg);

            // ===== TELEMETRY =====
            telemetry.addData("Mode", mode);
            telemetry.addData("HeadingFilt", heading);
            telemetry.addData("TurretDeg", turretDeg);
            telemetry.addData("TurretTicks", turretTicks);
            telemetry.addData("WorldNow", worldNow);
            telemetry.addData("WorldTargetAngle", worldTargetAngle);
            telemetry.addData("HasWorldTarget", hasWorldTarget);
            telemetry.addData("HasTag24", hasTag24);
            telemetry.addData("LastSeenTagID", lastSeenId);
            telemetry.addData("AutoTrack (LT)", autoTrack);
            telemetry.addData("TurretPower", turretPower);
            telemetry.update();
        }

        // stop all motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        turret.setPower(0);
        outtake.setPower(0);
    }

    // ===== HELPERS =====

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
}
