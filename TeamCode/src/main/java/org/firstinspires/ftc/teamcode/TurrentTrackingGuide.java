package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.lang.annotation.ElementType;

public class TurrentTrackingGuide {
    private DcMotorEx turret;

    private double kP = 0.00001;
    private double kD = 0;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final double MAX_POWER = 0.3;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setKp(double newKp) {
        kP = newKp;
    }

    public double getKp() {
        return kP;
    }
    public void setkD(double newKd) {
        kD = newKd;
    }

    public double getKD() {
        return kD;
    }
    public void resetTimer(){
        timer.reset();
    }
    // Change the parameter from AprilTagDetection to double tx
    public void update(double tx) {
        double deltaTime = timer.seconds();
        timer.reset();

        // With Limelight, tx IS the bearing.
        // Logic: How far are we (tx) from where we want to be (goalX)?
        double error = goalX - tx;

        double pTerm = error * kP;
        double dTerm = 0;

        if (deltaTime > 0) {
            dTerm = (error - lastError) / deltaTime * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            // Use your maxPower variable here
            power = com.qualcomm.robotcore.util.Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;
    }


}
