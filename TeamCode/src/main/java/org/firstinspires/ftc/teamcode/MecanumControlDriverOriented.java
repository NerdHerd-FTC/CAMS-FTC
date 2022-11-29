package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
public class MecanumControlDriverOriented extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motors (F=front, B=back, R=right, L=left)
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");


        // Right motors should move in reverse
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        // Left motors should move forward
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Y stick is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double theta = gamepad1.right_stick_x; //Rotate by theta

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(theta), 1);
            //Set motor powers
            double FLPower = (y + x + theta) / denominator;
            double BLPower = (y - x + theta) / denominator;
            double FRPower = (y - x - theta) / denominator;
            double BRPower = (y + x - theta) / denominator;

            //Run motors using powers
            motorFL.setPower(FLPower);
            motorBL.setPower(BLPower);
            motorFR.setPower(FRPower);
            motorBR.setPower(BRPower);
        }
    }
}


