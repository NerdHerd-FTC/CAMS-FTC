package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * AD 12/2/22
 */

@TeleOp(name="Robot: Claw Teleop", group="Robot")
//@Disabled
public class ClawTestTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    public Servo finger;
    public Servo palm;
    public Servo wrist;

    @Override
    public void runOpMode() {

        // Define and Initialize Hardware
        finger = hardwareMap.get(Servo.class, "ServoFinger");
        palm = hardwareMap.get(Servo.class, "ServoPalm");
        wrist = hardwareMap.get(Servo.class, "ServoWrist");

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double fingerTargetPos = 0.7;
        double palmTargetPos = 0.2;
        double wristTargetPos = 0.3;

        while (opModeIsActive()) {

            // finger
            if(gamepad2.left_bumper) {
                fingerTargetPos = 0;
            }
            else if(gamepad2.right_bumper) {
                fingerTargetPos = 1;
            }

            if(fingerTargetPos >= 1.0) {
                fingerTargetPos = 1.0;
            }
            else if(fingerTargetPos <= 0.0) {
                fingerTargetPos = 0.0;
            }

            /*
            // palm
            if(gamepad2.right_stick_x > 0) {
                palmTargetPos += 0.05;
            }
            else if(gamepad2.right_stick_x < 0) {
                palmTargetPos -= 0.05;
            }

            if(palmTargetPos >= 1.0) {
                palmTargetPos = 1.0;
            }
            else if(palmTargetPos <= 0.0) {
                palmTargetPos = 0.0;
            }
            */


            // wrist
            if(gamepad2.dpad_up) {
                wristTargetPos += 0.05;
            }
            else if(gamepad2.dpad_down) {
                wristTargetPos -= 0.05;
            }

            if(wristTargetPos >= 1.0) {
                wristTargetPos = 1.0;
            }
            else if(wristTargetPos <= 0.0) {
                wristTargetPos = 0.0;
            }


            finger.setPosition(fingerTargetPos);
            palm.setPosition(palmTargetPos);
            wrist.setPosition(wristTargetPos);
            telemetry.addData("fingerTargetPos", "%.2f", fingerTargetPos);
            telemetry.addData("palmTargetPos", "%.2f", palmTargetPos);
            telemetry.addData("wristTargetPos", "%.2f", wristTargetPos);
            telemetry.update();


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }

}
