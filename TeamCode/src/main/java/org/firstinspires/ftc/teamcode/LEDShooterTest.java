package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LED_Shooter_Test_Only", group = "Test")
public class LEDShooterTest extends LinearOpMode {

    private DcMotorEx outtake;
    private RevBlinkinLedDriver led;

    // --- SETTINGS ---
    private final double TARGET_VELOCITY = 2000.0;
    private final double TOLERANCE = 50.0;
    Servo kick1;
    Servo kick2;
    Servo kick3;
    @Override
    public void runOpMode() {
        // Hardware Mapping
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        kick3 = hardwareMap.get(Servo.class, "kick3");



        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready - Press Right Trigger to Spin Shooter");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean shootEnabled = gamepad1.right_trigger > 0.2;

            if (shootEnabled) {
                outtake.setVelocity(TARGET_VELOCITY);

                // Check if we are at speed
                double currentVel = outtake.getVelocity();
                if (Math.abs(currentVel - TARGET_VELOCITY) < TOLERANCE) {
                    // Force SOLID LIME (Green)
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                    if (gamepad1.a)
                        kick1.setPosition(0.7);
                    if (gamepad1.y)
                        kick2.setPosition(0.95);
                    if (gamepad1.x)
                       kick3.setPosition(0.45);
                } else {
                    // Force SOLID RED
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
                    kick1.setPosition(0.1);
                    kick2.setPosition(0.1);
                    kick3.setPosition(1);
                }

                telemetry.addData("Target", TARGET_VELOCITY);
                telemetry.addData("Actual", currentVel);
            } else {
                outtake.setVelocity(0);
                // When off, set to a neutral color (Blue) so we know it's working
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                telemetry.addLine("Shooter Off - LED should be BLUE");
            }

            telemetry.update();
        }
    }
}