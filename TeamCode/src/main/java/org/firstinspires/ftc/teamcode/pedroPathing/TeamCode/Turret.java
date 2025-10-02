package org.firstinspires.ftc.teamcode.pedroPathing.TeamCode;
import android.sax.StartElementListener;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.sun.source.doctree.StartElementTree;

import androidx.annotation.NonNull;




public class Turret {
    private Robot robot;
    private double target;
    private boolean speed_control = false;
    private double max_speed = 0.5; //0.1
    private double threshold = 0.005;
    private final double P = 0.03;

    private double cur_pos = 0.0;

    public Turret(Robot robot) {
        this.robot = robot;
        cur_pos = robot.servoTurret.getPosition();
        target = cur_pos;
    }
    public double getCurPos()
    {
        return robot.servoTurret.getPosition();
    }
    private void setSCTarget(double target) {
        speed_control = true;
        this.target = target;
    }

        //robot.telemetry.addData("Arm Curr Pos:", robot.servoArmLeft.getPosition());
        //robot.telemetry.addData("Arm Target:", this.target);
        //robot.telemetry.addData("Arm Speed Control: ", speed_control);
    }




