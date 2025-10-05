package org.firstinspires.ftc.teamcode.pedroPathing.TeamCode;

import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ShooterTester", group = "Examples")
public class ShooterTester extends OpMode {
//    public Robot robot, telemetry;

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
        Servo rawTurret = hardwareMap.get(Servo.class, "servoTurret");
        turret = new Turret(rawTurret);
        turret.setTarget(0.5);
        turret.apply();

    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_y != 0){ // code for shooter motor
//            double targetRPM = 5000.0;
//            double targetTPS = (targetRPM / 60.0) * 100;
//            DcMotorShooter.setPower(-gamepad1.left_stick_y);
            DcMotorShooter.setPower(-gamepad1.left_stick_y);

        }
        double currentVelocity = DcMotorShooter.getVelocity();
        telemetry.addData("Current RPM: ", currentVelocity);
//        telemetry.update();
         if (gamepad1.a) {
            DcMotorShooter.setVelocity(1460);
//         } else if (gamepad1.b) {
//             DcMotorShooter.setVelocity(1400);
         } else if (gamepad1.x) {
             DcMotorShooter.setVelocity(1000);
         } else if (gamepad1.y) {
             DcMotorShooter.setVelocity(1200);
         }

//        else if (gamepad1.b) {
//            BallShooterNear();
//        }
        if (gamepad1.dpad_left){
            currentTurretPose += incTurret;
        }
        else if (gamepad1.dpad_right) {
            currentTurretPose -= incTurret;
        }
//        turret.setTarget(currentTurretPose);
//        turret.apply();
        telemetry.addData("Increment:", incTurret);
        telemetry.addData("Arm Current Value:", currentTurretPose);
        telemetry.update();

    }
}
