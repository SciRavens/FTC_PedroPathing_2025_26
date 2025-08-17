package org.firstinspires.ftc.teamcode.pedroPathing.TeamCode;

import com.pedropathing.follower.*;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.TeamCode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "RobotTeleop", group = "Examples")
public class RobotTeleop extends OpMode {
    public Robot robot;
    //public DriveTrain DT;
    public Slider slider;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
//    public ClawAngle clawAngle;
    private Follower follower;
//    RevBlinkinLedDriver.BlinkinPattern pattern;
    private double slider_pos;
    private static final double Kp = 0.02;
    private static final double Ki = 0.0;
    private static final double Kd = 0.002;

    double strafeHeading; // for strafing straight
    boolean strafeHeadingOn = false;

    private static double Kp_strafing = 0.002;
    private static double DEAD_ZONE = 0.1;

    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPath();
    }

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(0,0,0));
        follower.startTeleopDrive();
        slider = new Slider(robot);
        arm = new Arm(robot);
        wrist = new Wrist(robot);
        claw = new Claw(robot);

        arm.setPosStarting(false);
        wrist.setPosStarting(false);
//        clawAngle.setHorizontal();
//        waitForStart();
//        while (opModeIsActive()) {
//            follower_operate();
//            arm.operate();
//            wrist.operate();
//            slider_joystick();
//            arm_wrist_operate();
//            claw_operate();
        robot.telemetry.update();
    }

    private void follower_operate() {
        boolean strafeOnly = false;
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? gamepad1.left_stick_y : 0;

        if (gamepad1.right_trigger > 0.5) {
            follower.setMaxPower(0.25);
        } else if (gamepad1.left_trigger > 0.5) {
            strafeOnly = true;
            if (!strafeHeadingOn) {
                strafeHeading = follower.getPose().getHeading();
            }
            // Heading correction
            double imuHeading = follower.getPose().getHeading();
            double headingError = strafeHeading - imuHeading; // Target heading is initialized when strafing starts
            double correction = Kp_strafing * headingError;

            // Adjust movement with IMU and drift correction
            follower.setTeleOpMovementVectors(xInput, yInput, correction);
        } else {
            follower.setMaxPower(1.0);
        }
        if (!strafeOnly) {
            strafeHeadingOn = false;
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
        follower.update();

        if (gamepad2.dpad_down) {
            arm.setPosSampleTwo(false);
            wrist.setPosSampleTwo(false);
        } else if (gamepad2.y) {
            arm.setPosBasket(false);
            wrist.setPosBasket(false);
//                clawAngle.setHorizontal();
//                slider.HighBasket();
        } else if (gamepad2.x) {
//                clawAngle.setHorizontal();
            arm.setPosStarting(false);
            wrist.setPosStarting(false);
//                slider.InitialPose();
        } else if (gamepad2.b) {
            arm.setPosSpecimen(false);
            wrist.setPosSpecimen(false);
        } else if (gamepad2.a) {
            arm.setPosSample(false);
            wrist.setPosSample(false);
        } else if (gamepad2.dpad_left) {
//                clawAngle.setHorizontal();
        } else if (gamepad2.dpad_right) {
//                clawAngle.setVertical();
        } else if (gamepad2.dpad_up) {
//                slider.LowChamber();
            arm.setPosChamberBack(false);
            wrist.setPosChamberBack(false);
        }
    }

    public Pose testPath = new Pose(-40,20,Math.toRadians(90));
    public PathChain randomPath;

    public void buildPath() {
        randomPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(testPath)))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), testPath.getHeading())
                        .build();
    }


    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
        if(gamepad1.a) {
            buildPath();
            follower.followPath(randomPath,true);
        }
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: truei
        */
        if(gamepad1.a) {
            buildPath();
            follower.followPath(randomPath,true);
        }

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}