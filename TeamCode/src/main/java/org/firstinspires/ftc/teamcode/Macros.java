/*
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**Developing lift and claw macros for easy, consistent scoring*/
@TeleOp(name= "Macros", group="Robot")
@Disabled
public class Macros extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  RVAMotor1   = null;
    public DcMotor  RVAMotor2  = null;

    //public Servo clawFinger = null;

    private ElapsedTime speed_mult_runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    //Core Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 4.167;   //gear ratio of gears and sprockets (125t/15t * 20t/40t = 4.167)
    static final double     FINAL_GEAR_DIAMETER_INCH     = 1.60;    //diameter of starting, 15t gear (CHECK THIS NUMBER!!)
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (FINAL_GEAR_DIAMETER_INCH * Math.PI); //ticks for motor to get one revolution at first gear * number of revolutions needed to have one revolution at output/output gear's circumference in inches (calculates number of ticks needed to get one inch of output)
    static final double     STARTING_INCHES = 16; //CHECK THIS NUMBER - RVA elevated to height

    static final double     INCHES_TO_REACH = 33.5 - STARTING_INCHES; //goes to high junction - adjust this number for claw
    static final double     TICKS_TO_REACH = INCHES_TO_REACH * COUNTS_PER_INCH;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double SPEED_MULT = 0.5;

        double ArmPower;
        double fingerTarget = 0.7;
        double wristTarget = 0.3;
        final double CLAW_SPEED = 0.05; //strictly less than 1

        boolean lifting = false;
        boolean dropping = false;
        double targetLiftLocation = 0;

        //Telemetry update variables:
        String speed = "Normal";
        String fingerPos = "Closed";

        // Define and Initialize Motors and Servos
        leftDrive  = hardwareMap.get(DcMotor.class, "MotorA");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorB");

        RVAMotor1  = hardwareMap.get(DcMotor.class, "MotorC");
        RVAMotor2 = hardwareMap.get(DcMotor.class, "MotorD");

        //clawFinger = hardwareMap.get(Servo.class, "ServoFinger");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        RVAMotor1.setDirection(DcMotor.Direction.FORWARD);
        RVAMotor2.setDirection(DcMotor.Direction.REVERSE);

        RVAMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RVAMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RVAMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RVAMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RVAMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RVAMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //clawFinger.setPosition(0.7);

        speed_mult_runtime.reset();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
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

            //Handle speed multiplication
            if (gamepad1.a && speed_mult_runtime.seconds() >= 2) {
                speed_mult_runtime.reset();
                if (SPEED_MULT <= 0.25) {
                    SPEED_MULT = 0.5;
                    speed = "Normal";
                }
                else {
                    SPEED_MULT = 0.25;
                    speed = "Slow";
                }
            }

            //RVA Macro!

            //set power to zero when motors are off
            if (!RVAMotor1.isBusy() && !RVAMotor2.isBusy()) {
                RVAMotor1.setPower(0);
                RVAMotor2.setPower(0);
            }

            //macro
            if (gamepad1.b) { //kill switch!
                RVAMotor1.setPower(0);
                RVAMotor2.setPower(0);
            } else if (gamepad1.a) { //go up
                RVAMotor1.setTargetPosition((int) TICKS_TO_REACH);
                RVAMotor2.setTargetPosition((int) TICKS_TO_REACH);

                RVAMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RVAMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                RVAMotor1.setPower(0.65);
                RVAMotor2.setPower(0.65);
            } else if (gamepad1.y) { //go down
                RVAMotor1.setTargetPosition(0);
                RVAMotor2.setTargetPosition(0);

                RVAMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RVAMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                RVAMotor1.setPower(0.65);
                RVAMotor2.setPower(0.65);
            }

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
            leftDrive.setPower(Math.pow(left, 3) * SPEED_MULT);
            rightDrive.setPower(Math.pow(right, 3) * SPEED_MULT);

            //HANDLE CLAW

            //Handle claw open and close
            //if (gamepad1.right_bumper) {
                //clawFinger.setPosition(0); //close
                //fingerPos = "Closed";
            //}
            //else if (gamepad1.left_bumper){
                //clawFinger.setPosition(1); //open
                //fingerPos = "Open";
            //}

            //Raise or lower claw
            //if (gamepad1.dpad_up){
                //wristTarget -= CLAW_SPEED;
            //}
            //else if (gamepad1.dpad_down){
                //wristTarget += CLAW_SPEED;
            //}
            //clawWrist.setPosition(wristTarget);

            // Send telemetry message to signify robot running;
            telemetry.addData("Speed: ", "String", speed);
            telemetry.addData("Stick X: ",  "%.2f", turn);
            telemetry.addData("Stick Y: ", "%.2f", (drive * -1));
            //telemetry.addData("Fingers are: ", fingerPos);
            telemetry.addData("RVA Motor A Encoder: %7d", RVAMotor1.getCurrentPosition());
            telemetry.addData("RVA Motor B Encoder: %7d", RVAMotor2.getCurrentPosition());
            telemetry.addData("\nRVA Motor A TARGET: %7d", RVAMotor1.getTargetPosition());
            telemetry.addData("RVA Motor B TARGET: %7d", RVAMotor2.getTargetPosition());
            telemetry.update();


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}