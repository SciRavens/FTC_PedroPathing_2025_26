package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        DriveEncoderConstants.forwardTicksToInches = 1;
        DriveEncoderConstants.strafeTicksToInches = 1;
        DriveEncoderConstants.turnTicksToInches = 1;

        DriveEncoderConstants.robot_Width = 12.75;
        DriveEncoderConstants.robot_Length = 15.5;


        ThreeWheelConstants.forwardTicksToInches = 0.0019916228544508514;
        ThreeWheelConstants.strafeTicksToInches = 0.0019971702092947082;
        ThreeWheelConstants.turnTicksToInches = 0.002150436788393751;
        ThreeWheelConstants.leftY = 4.125;
        ThreeWheelConstants.rightY = -4.125;
        ThreeWheelConstants.strafeX = -3.5;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftRear";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightRear";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;//changed
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;//changed
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




