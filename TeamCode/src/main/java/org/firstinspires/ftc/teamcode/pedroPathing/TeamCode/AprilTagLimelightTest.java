package org.firstinspires.ftc.teamcode.pedroPathing.TeamCode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.TeamCode.AprilTagLimelightTest;

public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    private double distance;
    private IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // april tag pipeline
        RevHubOrientationOnRobot revHubOrientationOnRobot= new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();//change if there is a delay, run in init statement
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()){
            Pose3D botpose = llresult.getBotpose_MT2();
            distance = getDistanceFromTags(llresult.getTa());
            telemetry.addData("ditance", distance);
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Tx", llresult.getTy());
            telemetry.addData("TA", llresult.getTa());
        }
    }

    public double getDistanceFromTags(double ta) {
        double scale = 30000; //placeholder value
        double distance = scale/ta;
        return distance;
    }
}
