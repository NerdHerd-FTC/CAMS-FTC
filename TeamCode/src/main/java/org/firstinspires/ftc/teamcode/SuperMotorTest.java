package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

//Basically a motor test, but it can test any number of motors up to 4
@TeleOp(name="Super Motor Test", group="Robot")
@Disabled
public class SuperMotorTest extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor Motor1   = null;
    public DcMotor Motor2 = null;
    public DcMotor Motor3 = null;
    public DcMotor Motor4 = null;

    @Override
    public void runOpMode() {
        double power;
        double max;
        final int control = 25;
        //Control (min 1) gives the driver more control over the free speed motor.

        List<DcMotor> connectedMotors = new ArrayList<>();
        // Define and Initialize Motors
        try {
            Motor1  = hardwareMap.get(DcMotor.class, "MotorA");
            connectedMotors.add(Motor1);
        } catch (Exception e) {}
        try {
            Motor2  = hardwareMap.get(DcMotor.class, "MotorB");
            connectedMotors.add(Motor2);
        } catch (Exception e) {}
        try {
            Motor3  = hardwareMap.get(DcMotor.class, "MotorC");
            connectedMotors.add(Motor3);
        } catch (Exception e) {}
        try {
            Motor4  = hardwareMap.get(DcMotor.class, "MotorD");
            connectedMotors.add(Motor4);
        } catch (Exception e) {}

        for (DcMotor motor: connectedMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            telemetry.addData("Connected to Motor at Port", "%d", motor.getPortNumber());
        }

        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addLine("Motor test in progress. Use the left stick to provide power.");

            // Normalize the values so neither exceed +/- 1.0
            power = gamepad1.left_stick_y;

            max = Math.abs(Math.pow(power, 2 * control - 1));
            if (max > 1.0)
            {
                power /= max;
            }

            telemetry.addData("Power",  "%.2f", power);
            for (DcMotor motor: connectedMotors) {
                motor.setPower(power);
                telemetry.addLine(String.format("Encoder for Motor %s: %s", motor.getPortNumber(), motor.getCurrentPosition()));
            }

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}