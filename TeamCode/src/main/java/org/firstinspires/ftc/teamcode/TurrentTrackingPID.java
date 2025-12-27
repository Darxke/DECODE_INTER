package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TurrentTrackingPID", group = "Test")
public class TurrentTrackingPID extends LinearOpMode {

    // turret + drive
    private DcMotorEx turret;
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private IMU imu;

    // encoder to degrees
    private static final double TICKS_PER_DEGREE = 4.44;

    // signs to fix directions if needed
    private static final double TURRET_SIGN = 1.0;   // flip to -1.0 if turret direction is reversed
    private static final double HEADING_SIGN = -1.0; // flip to 1.0 if turret fights your turn

    // world lock stuff
    private double lockWorldAngle = 0.0;
    private boolean hasLock = false;

    // filtered IMU heading
    private double headingFiltered = 0.0;

    // simple P control on angle error
    private static final double K_AIM = 0.03;
    private static final double MAX_TURRET_POWER = 0.8;
    private static final double ANGLE_DEADBAND_DEG = 1.5;

    @Override
    public void runOpMode() throws InterruptedException {

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        imu = hardwareMap.get(IMU.class, "imu");

        // IMU
        imu.resetYaw();
        headingFiltered = getSignedHeading();

        // drivetrain directions like AprilTurnRed
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // turret setup
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("TurrentTracking (IMU only)");
        telemetry.addLine("A = lock world direction");
        telemetry.addLine("LT = track lock, LB/RB = manual turret");
        telemetry.update();

        waitForStart();

        // take initial lock straight ahead at start
        double heading = headingFiltered;
        double turretDeg = getTurretDeg();
        lockWorldAngle = wrapAngle(heading + turretDeg);
        hasLock = true;

        while (opModeIsActive()) {

            // filtered heading
            double rawHeading = getSignedHeading();
            double alpha = 0.3;
            double delta = wrapAngle(rawHeading - headingFiltered);
            headingFiltered = wrapAngle(headingFiltered + alpha * delta);
            heading = headingFiltered;

            turretDeg = getTurretDeg();
            int turretTicks = turret.getCurrentPosition();

            // DRIVE - gamepad1
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

            // lock with A
            if (gamepad1.a) {
                turretDeg = getTurretDeg();
                lockWorldAngle = wrapAngle(heading + turretDeg);
                hasLock = true;
            }

            boolean autoTrack = hasLock && gamepad1.left_trigger > 0.2;
            double turretPower = 0.0;

            if (autoTrack) {
                // desired turret angle = world lock minus current heading
                double desiredTurretDeg = wrapAngle(lockWorldAngle - heading);

                // wrapped error so it always takes shortest path
                double error = wrapAngle(desiredTurretDeg - turretDeg);

                if (Math.abs(error) < ANGLE_DEADBAND_DEG) {
                    turretPower = 0.0;
                } else {
                    turretPower = K_AIM * error;
                    if (turretPower > MAX_TURRET_POWER) turretPower = MAX_TURRET_POWER;
                    if (turretPower < -MAX_TURRET_POWER) turretPower = -MAX_TURRET_POWER;
                }

            } else {
                // manual turret
                double bumpPower = 0.35;
                if (gamepad1.left_bumper) {
                    turretPower = -bumpPower;
                } else if (gamepad1.right_bumper) {
                    turretPower = bumpPower;
                } else {
                    turretPower = 0;
                }
            }

            turret.setPower(turretPower);

            double worldNow = wrapAngle(heading + turretDeg);

            telemetry.addData("Heading filt", heading);
            telemetry.addData("Turret deg", turretDeg);
            telemetry.addData("Turret ticks", turretTicks);
            telemetry.addData("LockWorld", lockWorldAngle);
            telemetry.addData("World now", worldNow);
            telemetry.addData("AutoTrack (LT)", autoTrack);
            telemetry.addData("SlowMode (RB)", slowMode);
            telemetry.addData("TurretPower", turretPower);
            telemetry.update();
        }

        // stop everything
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        turret.setPower(0);
    }

    // helpers

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
