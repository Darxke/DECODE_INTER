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

    // ====== INTAKE ======
    private DcMotor intake;

    // ====== KICKERS / SERVOS ======
    private Servo kick1;
    private Servo kick2;
    private Servo kick3;

    private static final double KICK_REST_POS = 0.0;
    private static final double KICK_FIRE_POS = 1.0;

    // ====== IMU ======
    private IMU imu;
    private static final double HEADING_SIGN = -1.0;  // flip if heading backwards

    // turret encoder → degrees
    private static final double TICKS_PER_DEGREE = 4.44;
    private static final double TURRET_SIGN = 1.0;   // flip to -1.0 if turret angle inverted

    private double headingFiltered = 0.0;
    private static final double HEADING_FILTER_ALPHA = 0.3;

    // ====== WORLD LOCK (last seen direction of the goal) ======
    private double worldTargetAngle = 0.0;
    private boolean hasWorldTarget = false;   // starts false

    // world-lock gains (stronger again so it actually tracks)
    private static final double K_AIM_WORLD = 0.05;
    private static final double MAX_TURRET_POWER_WORLD = 0.7;
    private static final double ANGLE_DEADBAND_DEG = 2.0;

    // ====== LIMELIGHT ======
    private Limelight3A limelight;

    private static final int TARGET_TAG_ID = 24;

    // Limelight PID (still soft to avoid shaking)
    private static final double LIMELIGHT_KP_TURN = 0.015;
    private static final double LIMELIGHT_MAX_TURN = 0.6;
    private static final double LIMELIGHT_AIM_TOLERANCE = 2.0; // bigger deadband

    // camera-to-shooter offset in degrees
    // negative = bias to the RIGHT. You said it aims left, so we shove more right.
    private static final double AIM_OFFSET_DEG = -7.0;

    // low-pass filter for tx
    private double lastTx = 0.0;
    private double txFiltered = 0.0;
    private static final double TX_FILTER_ALPHA = 0.3;

    // ====== SHOOTER DISTANCE → RPM ======
    private static final double TICKS_PER_REV = 28.0;   // goBILDA 435rpm motor
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;

    // new request: very weak close, stronger far
    private static final double FAR_RPM   = 3750.0;  // far shot
    private static final double CLOSE_RPM = 1000.0;  // close shot

    // rough LL area range
    private static final double AREA_FAR   = 1.0;
    private static final double AREA_CLOSE = 8.0;

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

        // ===== INTAKE =====
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD); // flip if needed

        // ===== KICKERS =====
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");


        // ===== IMU =====
        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
        headingFiltered = getSignedHeading();

        // ===== LIMELIGHT =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        telemetry.addLine("TurrentTracking (LL + IMU world lock + distance shooter)");
        telemetry.addLine("Hold LT = track ID 24; when lost, aim last world angle (after first lock)");
        telemetry.addLine("Hold RT = shooter (velocity from tag distance)");
        telemetry.addLine("A = intake, LB/RB = manual turret, Y/B/X = servos");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === IMU filtered heading ===
            double rawHeading = getSignedHeading();
            double dHead = wrapAngle(rawHeading - headingFiltered);
            headingFiltered = wrapAngle(headingFiltered + HEADING_FILTER_ALPHA * dHead);
            double heading = headingFiltered;

            // turret angle
            double turretDeg = getTurretDeg();

            // === drive ===
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;
            driveRobot(drive, strafe, turn);

            // intake on A
            intake.setPower(gamepad1.a ? 1.0 : 0.0);

            // servos
            kick1.setPosition(gamepad1.y ? 1: KICK_REST_POS);
            // B servo flipped: default FIRE, press B → REST
            kick2.setPosition(gamepad1.b ? 0.3 : KICK_FIRE_POS);
            kick3.setPosition(gamepad1.x ? 0.1 : 0.9);

            // === Limelight: STRICTLY ID 24 ===
            LLResult result = limelight.getLatestResult();
            boolean hasTag24Now = false;
            double txNow = lastTx;
            double areaNow = lastTargetArea;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                if (fids != null && !fids.isEmpty()) {
                    for (LLResultTypes.FiducialResult fid : fids) {
                        if (fid.getFiducialId() == TARGET_TAG_ID) {
                            txNow = fid.getTargetXDegrees();
                            areaNow = fid.getTargetArea();
                            hasTag24Now = true;
                            break;
                        }
                    }
                }
            }

            if (hasTag24Now) {
                lastTx = txNow;
                lastTargetArea = areaNow;

                // smooth tx
                double txErr = txNow - txFiltered;
                txFiltered += TX_FILTER_ALPHA * txErr;

                // ALWAYS update worldTargetAngle when we see the tag
                worldTargetAngle = wrapAngle(heading + turretDeg);
                hasWorldTarget = true;
            }

            // === turret control ===
            boolean autoAim = gamepad1.left_trigger > 0.2;
            double turretPower = 0.0;
            String mode = "Idle";

            if (autoAim) {
                if (hasTag24Now) {
                    // ==== LIMELIGHT DIRECT TRACKING ON ID 24 ====
                    double error = txFiltered + AIM_OFFSET_DEG;

                    if (Math.abs(error) < LIMELIGHT_AIM_TOLERANCE) {
                        turretPower = 0.0;
                    } else {
                        turretPower = LIMELIGHT_KP_TURN * error;
                        if (turretPower > LIMELIGHT_MAX_TURN) turretPower = LIMELIGHT_MAX_TURN;
                        if (turretPower < -LIMELIGHT_MAX_TURN) turretPower = -LIMELIGHT_MAX_TURN;
                    }

                    mode = "LL tracking ID24";

                } else if (hasWorldTarget) {
                    // ==== IMU WORLD LOCK WHEN TAG IS LOST ====
                    double turretWorldNow = wrapAngle(heading + turretDeg);
                    double errorWorld = wrapAngle(worldTargetAngle - turretWorldNow);

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
                    mode = "Auto, no lock yet";
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
                if (lastTargetArea > 0.001) {
                    rpm = computeRpmFromArea(lastTargetArea);
                } else {
                    // fallback when no tag info
                    rpm = 2200.0;
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
            telemetry.addData("HasTag24Now", hasTag24Now);
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
        intake.setPower(0.0);
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
        double a = clamp(area, AREA_FAR, AREA_CLOSE);
        double t = (a - AREA_FAR) / (AREA_CLOSE - AREA_FAR);
        return FAR_RPM + t * (CLOSE_RPM - FAR_RPM);
    }
}
