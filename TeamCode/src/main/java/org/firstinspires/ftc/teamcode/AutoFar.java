package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.MecanumDrive; // RR 1.0 quickstart drive

@Config
@Autonomous(name = "RRFarBlue", group = "Autonomous")
public class AutoFar extends LinearOpMode {
    private DcMotorEx rightOut;

    private DcMotorEx leftOut;
    private DcMotorEx intake;
    private Servo kick1;
    private Servo kick2;


    private static double open1 = 0.4;
    private static double close1 = 0;
    private static double open2 = 0.6;
    private static double close2 = 0.3;
    private static double push = 1;
    private static double store = 0.5;
    @Override
    public void runOpMode() {
        // Start pose — place robot here. Change if your field frame is different.
        rightOut = hardwareMap.get(DcMotorEx.class, "rightOut");
        leftOut = hardwareMap.get(DcMotorEx.class, "leftOut");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        leftOut.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightOut.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightOut.setDirection(DcMotorSimple.Direction.REVERSE);
        Pose2d start = new Pose2d(59, -12, Math.toRadians(195));
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        Action shoot = drive.actionBuilder(new Pose2d(32, -64, Math.toRadians(260)))
                .strafeToLinearHeading(new Vector2d(59, -12), Math.toRadians(195))
                .build();

        Action parking = drive.actionBuilder(new Pose2d(59, -12, Math.toRadians(195)))
                .strafeToLinearHeading(new Vector2d(35, -20), Math.toRadians(180))
                .build();

        Action out = drive.actionBuilder(new Pose2d(59, -12, Math.toRadians(195)))   // -35° → 325°
                .strafeToLinearHeading(new Vector2d(32, -30), Math.toRadians(260))
                .build();

        Action cycle = drive.actionBuilder(new Pose2d(32, -30, Math.toRadians(260)))
                .strafeToLinearHeading(
                        new Vector2d(32, -64),
                        Math.toRadians(260)
//                        new TranslationalVelConstraint(20.0),
//                        new ProfileAccelConstraint(-10.0, 10.0)
                )
                .build();
        Action out2 = drive.actionBuilder(new Pose2d(59, -12, Math.toRadians(195)))   // -35° → 325°
                .strafeToLinearHeading(new Vector2d(9, -31), Math.toRadians(260))
                .build();

        Action cycle2 = drive.actionBuilder(new Pose2d(9, -31, Math.toRadians(260)))
                .strafeToLinearHeading(
                        new Vector2d(9, -64),
                        Math.toRadians(260)
//                        new TranslationalVelConstraint(20.0),
//                        new ProfileAccelConstraint(-10.0, 10.0)
                )
                .build();
        Action shoot2 = drive.actionBuilder(new Pose2d(9, -64, Math.toRadians(260)))
                .strafeToLinearHeading(new Vector2d(59, -12), Math.toRadians(195))
                .build();


        telemetry.addLine("Ready (RR 1.0). Set robot at start pose.");
        telemetry.update();
        waitForStart();
        leftOut.setVelocity(1060);
        rightOut.setVelocity(1060);
        intake.setPower(1);
        if (isStopRequested()) return;
        sleep(2250);
        shooting();
        sleep(250);
        //    shooting();
        Actions.runBlocking(out);
        sleep(250);
        Actions.runBlocking(cycle);
        sleep(250);
        leftOut.setVelocity(1060);
        rightOut.setVelocity(1060);
        Actions.runBlocking(shoot);
        intake.setPower(1);
        shooting();
        sleep(250);
        Actions.runBlocking(out2);
        sleep(250);
        Actions.runBlocking(cycle2);
        sleep(500);
        leftOut.setVelocity(1060);
        rightOut.setVelocity(1060);
        Actions.runBlocking(shoot2);
        intake.setPower(1);
        shooting();
        Actions.runBlocking(parking);
    }
    public void shooting(){
        kick1.setPosition(0);
        kick2.setPosition(1);
        sleep(750);
        kick2.setPosition(0);
        sleep(1000);
        kick2.setPosition(1);
        sleep(750);
        kick2.setPosition(0);
        sleep(750);
        kick1.setPosition(.4);
        sleep(250);
        kick1.setPosition(.4);
        kick2.setPosition(1);
        sleep(500);
        kick1.setPosition(0);
        kick2.setPosition(0);
    }
}
