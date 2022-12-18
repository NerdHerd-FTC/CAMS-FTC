package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 *
 */

@TeleOp(name="Robot: Teleop POV", group="Robot")
@Disabled
public class RobotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor FrontRightMotor = null;
    private DcMotor FrontLeftMotor  = null;

    // Control/Expansion Hub IMU
    private BNO055IMU imu = null;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // Define and Initialize Hardware
        FrontRightMotor  = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward,
            // so negate it). In this mode the Left stick moves the robot fwd and back,
            // the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives. The exponent gives the
            // driver better control over the free speed of the motor.
            FrontRightMotor.setPower(right);
            FrontLeftMotor.setPower(left);

            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    private double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
