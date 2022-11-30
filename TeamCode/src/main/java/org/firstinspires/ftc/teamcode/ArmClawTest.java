package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Arm and Claw Test", group="Robot")
public class ArmClawTest extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor Arm = null;

    @Override
    public void runOpMode() {
        double ArmPower = 0;

        Arm = hardwareMap.get(DcMotor.class, "MotorC");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Arm.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_trigger >= 0.4) {
                ArmPower = 0.5;
            } else if (gamepad1.left_trigger >= 0.4) {
                ArmPower = -0.5;
            }

            Arm.setPower(ArmPower);
            
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}