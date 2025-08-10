package org.firstinspires.ftc.teamcode.pedroPathing.examples;
import android.sax.StartElementListener;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.sun.source.doctree.StartElementTree;

import androidx.annotation.NonNull;
public class Arm {
    private Robot robot;
    private double target;
    private boolean speed_control = false;
    private double max_speed = 0.5; //0.1
    private double threshold = 0.005;
    private final double P = 0.03;

    private double cur_pos = 0.0;

    public Arm(Robot robot) {
        this.robot = robot;
        cur_pos = robot.servoArmLeft.getPosition();
        target = cur_pos;
    }
}
