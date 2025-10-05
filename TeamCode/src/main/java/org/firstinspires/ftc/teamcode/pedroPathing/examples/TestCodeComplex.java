package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Test Code Complex", group = "Examples")

public class TestCodeComplex extends OpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Follower follower;
    private int pathState;
    private final Pose startPose = new Pose(-7.75, 6.375, Math.toRadians(180));
    private final Pose forwardPose = new Pose(-82, 84, Math.toRadians(0));
    private final Pose forwardControlPose = new Pose(-75, 15, Math.toRadians(0));
    private final Pose backPose = new Pose(-8, 82, Math.toRadians(0));
    private final Pose secondSplinePose = new Pose(-84, 10, Math.toRadians(180));
    private final Pose secondSplineControlPose = new Pose(-10, 15, Math.toRadians(180));
    private final Pose finalBackPose = new Pose(-8, 6, Math.toRadians(180));


    private Path initialSpline;
    private Path finalBack;
    private PathChain backPath, finalSpline;

    public void buildPaths() {
//        startPath = new Path(new BezierLine(new Point(startPose), new Point(forwardPose)));
//        startPath.setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading());

        initialSpline = new Path(new BezierCurve(new Point(startPose), /* Control Point */ new Point(forwardControlPose), new Point(forwardPose)));
        initialSpline.setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading());
        backPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(forwardPose), new Point(backPose)))
                .setLinearHeadingInterpolation(forwardPose.getHeading(), backPose.getHeading())
                .build();
        finalSpline = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(backPose), /* Control Point */ new Point(secondSplineControlPose), new Point(secondSplinePose)))
                .setLinearHeadingInterpolation(backPose.getHeading(), secondSplinePose.getHeading())
                .build();
        finalBack = new Path(new BezierLine(new Point(secondSplinePose), new Point(finalBackPose)));
        finalBack.setLinearHeadingInterpolation(secondSplinePose.getHeading(), finalBackPose.getHeading());

//        park = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(forwardPose), new Point(turnPose)))
//                .setLinearHeadingInterpolation(forwardPose.getHeading(), turnPose.getHeading())
//                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(initialSpline);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(backPath);
                    setPathState(2);
                }
                    break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(finalSpline);
                    setPathState(3);
                }
                    break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath((finalBack));
                    setPathState(4);
                }
                    break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }
    @Override
    public void init_loop() {}
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop() {
    }
}
