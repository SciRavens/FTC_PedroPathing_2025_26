package org.firstinspires.ftc.teamcode.pedroPathing.TeamCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.pedropathing.follower.*;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.TeamCode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;

@TeleOp(name = "ShooterTester", group = "Examples")
public class ShooterTester extends OpMode {
    public Robot robot, telemetry;

    public DcMotorEx DcMotorShooter;
    public Turret turret;
    public double initialTurretPose = 0.5;
    public double currentTurretPose = initialTurretPose;
    public double incTurret = 0.25;
    private static double DEAD_ZONE = 0.1;


    public void BallShooterFar() {
        DcMotorShooter.setPower(0.6);
    }

    public void BallShooterNear() {
        DcMotorShooter.setPower(0.5);
    }

    @Override
    public void init() {

        DcMotorShooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        DcMotorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorShooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_y != 0){
//            double targetRPM = 5000.0;
//            double targetTPS = (targetRPM / 60.0) * 100;
//            DcMotorShooter.setPower(-gamepad1.left_stick_y);
            DcMotorShooter.setVelocity(-gamepad1.left_stick_y);
            double currentVelocity = DcMotorShooter.getVelocity();
            robot.telemetry.addData("Current RPM: ", currentVelocity);
            robot.telemetry.update();
        }
        else if (gamepad1.a) {
            BallShooterFar();
        }
        else if (gamepad1.b) {
            BallShooterNear();
        }
        else if (gamepad1.dpad_left){
            if (currentTurretPose < 0.9) {
                currentTurretPose += incTurret;
            }
        }
        else if (gamepad1.dpad_right) {
            if (currentTurretPose > 0.1) {
                currentTurretPose -= incTurret;
            }
            robot.telemetry.addData("Increment:", incTurret);
            robot.telemetry.addData("Arm Current Value:", currentTurretPose);
            robot.telemetry.update();

        }
    }
}
