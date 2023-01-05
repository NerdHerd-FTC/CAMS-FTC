package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Reset Yaw", group="Robot")
public class ResetYaw extends LinearOpMode {
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    IMU imu;

    @Override
    public void runOpMode() {
        //initialize & setup IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[0]; //logo facing UP
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[4]; //usb ports facing to the LEFT
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.setMsTransmissionInterval(20);

        //provide IMU data to user to determine if it is necessary to reset yaw
        while (!isStarted() && !isStopRequested()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            telemetry.addLine("MAKE SURE ROBOT IS CORRECTLY ORIENTATED BEFORE STARTING!");
            telemetry.addData("Current Yaw", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        imu.resetYaw(); //resets yaw
    }
}