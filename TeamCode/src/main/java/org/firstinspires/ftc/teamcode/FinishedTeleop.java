package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Driver-Operator integrated control
 */
@TeleOp(name= "Yee Yee Teleop", group="Robot")
public class FinishedTeleop extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  RVAMotor1   = null;
    public DcMotor  RVAMotor2  = null;

    public Servo finger = null;
    public Servo wrist = null;
    public Servo palm = null;

    public static final double MIN_POSITION  =  0.0 ;
    public static final double MAX_POSITION  =  0.5 ; //THIS IS THE NUMBER TO EDIT

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double speedMult = 0.5;
        double ArmPower;
        boolean buttonLock = false;



        //Telemetry Update Variables
        String fingerPos = "Closed";

        //Telemetry update variables:
        String speed = "Normal";

        // Define and Initialize Motors and Servos
        leftDrive  = hardwareMap.get(DcMotor.class, "MotorA");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorB");
        RVAMotor1  = hardwareMap.get(DcMotor.class, "MotorC");
        RVAMotor2 = hardwareMap.get(DcMotor.class, "MotorD");
        finger = hardwareMap.get(Servo.class, "ServoFinger");
        //palm = hardwareMap.get(Servo.class, "ServoPalm");
        wrist = hardwareMap.get(Servo.class, "ServoWrist");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        RVAMotor1.setDirection(DcMotor.Direction.FORWARD);
        RVAMotor2.setDirection(DcMotor.Direction.REVERSE);
        RVAMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RVAMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double fingerTargetPos = 0.7;
        double palmTargetPos = 0.2;
        double wristTargetPos = 0.3;

        // Send telemetry message to signify robot waiting;
        telemetry.addLine("Ready to start, good luck!");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -1 * gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;


            // finger
            if((gamepad2.left_bumper))
            {
                if(gamepad2.right_bumper) {
                    fingerTargetPos = 0;
                }
            }
            else if(gamepad2.right_bumper) {
                if(!gamepad2.left_bumper) {
                    fingerTargetPos = 1;
                }
            }

            /* Not used at the moment
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
            if(gamepad2.right_stick_y > 0.2) {
                wristTargetPos += 0.05;
            }
            else if(gamepad2.right_stick_y < -0.2) {
                wristTargetPos -= 0.05;
            }

            if(wristTargetPos >= 1.0) {
                wristTargetPos = 1.0;
            }
            else if(wristTargetPos <= 0.0) {
                wristTargetPos = 0.0;
            }

            finger.setPosition(fingerTargetPos);
            //palm.setPosition(palmTargetPos);
            wrist.setPosition(wristTargetPos);
            telemetry.addData("fingerTargetPos: ", "%.2f", fingerTargetPos);
            telemetry.addData("palmTargetPos: ", "%.2f", palmTargetPos);
            telemetry.addData("wristTargetPos: ", "%.2f", wristTargetPos);


            if(gamepad1.a) {
                if (speedMult <= 0.25) {
                    speedMult = 0.5;
                    speed = "Normal";
                } else {
                    speedMult = 0.25;
                    speed = "Slow";
                }
            }


            //Handle arm lowering/lifting
            if (gamepad1.left_trigger >= 0.4){
                ArmPower = -0.2;
            }
            else if (gamepad1.right_trigger >= 0.4){
                ArmPower = 0.4;
            }
            else {
                ArmPower = 0.1;
            }

            RVAMotor1.setPower(ArmPower);
            RVAMotor2.setPower(ArmPower);

            telemetry.addData("Arm1 Power: ", "%.2f", RVAMotor1.getPower());
            telemetry.addData("Arm2 Power: ", "%.2f", RVAMotor2.getPower());
            telemetry.addData("Arm1 Position: ", "%d", RVAMotor1.getCurrentPosition());
            telemetry.addData("Arm2 Position: ", "%d", RVAMotor2.getCurrentPosition());

            //Drive!
            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;
            // Normalize the values so neither exceed +/FinalControlScheme- 1.0
            if (left > 1.0) {
                left = 1.0;
            }
            if (right > 1.0){
                right = 1.0;
            }
            // Output the safe vales to the motor drives.
            leftDrive.setPower(Math.pow(left, 3) * speedMult);
            rightDrive.setPower(Math.pow(right, 3) * speedMult);

            // Send telemetry message to signify robot running;
            telemetry.addData("Speed: ", "String", speed);
            telemetry.addData("Stick X: ",  "%.2f", turn);
            telemetry.addData("Stick Y: ", "%.2f", (drive * -1));
            telemetry.update();
            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}