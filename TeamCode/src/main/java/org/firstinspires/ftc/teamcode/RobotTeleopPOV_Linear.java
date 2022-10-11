package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Robot: Teleop POV", group="Robot")
public class RobotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  MotorA  = null;
    public static final double MOTOR_POWER  = 0.1;

    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        MotorA  = hardwareMap.get(DcMotor.class, "MotorA");

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            MotorA.setPower(MOTOR_POWER);

            // Send telemetry message to signify robot running;
            telemetry.addLine("Running");
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }

        telemetry.addLine("Stopped");
        telemetry.update();
    }
}
