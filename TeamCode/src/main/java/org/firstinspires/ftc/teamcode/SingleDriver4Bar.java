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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TelelOp for Double Reverse Virtual 4 Bar with Macro
 */
@TeleOp(name= "Single Driver - RV4B", group="Robot")
public class SingleDriver4Bar extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor RV4BMotor1 = null;
    public DcMotor RV4BMotor2 = null;
    public Servo clawFinger = null;

    static final int  HIGH_JUNCTION_TICKS = -685;
    static final int  MEDIUM_JUNCTION_TICKS = -542;
    static final int  LOW_JUNCTION_TICKS = -400;
    
    final double K_P_TURN = 0.001;
    final double K_D_TURN = 0.03;
    final long DELTA_T = 20 + (long) telemetry.getMsTransmissionInterval();
    final double D_MULT_TURN = K_D_TURN / DELTA_T;

    static final double MACRO_POWER = 0.4; //for quick adjustments
    static final double ARM_POWER = 0.7; //prevent rogue negatives
    static final int ARM_SPEED_MANUAL = 15;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double degrees;
        double SPEED_MULT = 0.75;
        boolean xStorage = false;

        //Telemetry update variables:
        String speed = "Normal";
        String fingerPos = "Closed";

        // Define and Initialize Motors and Servos
        leftDrive  = hardwareMap.get(DcMotor.class, "MotorA");
        rightDrive = hardwareMap.get(DcMotor.class, "MotorB");
        RV4BMotor1 = hardwareMap.get(DcMotor.class, "MotorC");
        RV4BMotor2 = hardwareMap.get(DcMotor.class, "MotorD");
        clawFinger = hardwareMap.get(Servo.class, "ServoFinger");

        //core hex motors are facing opposite each other and will rotate in opposite directions
        RV4BMotor1.setDirection(DcMotor.Direction.FORWARD);
        RV4BMotor2.setDirection(DcMotor.Direction.REVERSE);

        RV4BMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RV4BMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              
        RV4BMotor1.setTargetPosition(0); 
        RV4BMotor2.setTargetPosition(0);

        RV4BMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RV4BMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RV4BMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RV4BMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        runtime.reset();

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
            drive = gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            //Handle speed multiplication
            if (gamepad1.x && !xStorage) {
                xStorage = true;
                if (SPEED_MULT <= 0.35) {
                    SPEED_MULT = 0.75;
                    speed = "Normal";
                }
                else {
                    SPEED_MULT = 0.35;
                    speed = "Slow";
                }
            }
            else{
                xStorage = false;
            }

            //Macros!
            if (gamepad1.y) { //go to high
                RV4BMotor1.setTargetPosition(HIGH_JUNCTION_TICKS);
                RV4BMotor2.setTargetPosition(HIGH_JUNCTION_TICKS);

                RV4BMotor1.setPower(MACRO_POWER);
                RV4BMotor2.setPower(MACRO_POWER);
            }
            else if (gamepad1.b) { //go to medium
                RV4BMotor1.setTargetPosition(MEDIUM_JUNCTION_TICKS);
                RV4BMotor2.setTargetPosition(MEDIUM_JUNCTION_TICKS);

                RV4BMotor1.setPower(MACRO_POWER);
                RV4BMotor2.setPower(MACRO_POWER);
            }
            else if (gamepad1.x) { //go to low
                RV4BMotor1.setTargetPosition(LOW_JUNCTION_TICKS);
                RV4BMotor2.setTargetPosition(LOW_JUNCTION_TICKS);

                RV4BMotor1.setPower(MACRO_POWER);
                RV4BMotor2.setPower(MACRO_POWER);
            }
            else if (gamepad1.a) { //go to ground
                RV4BMotor1.setTargetPosition(0);
                RV4BMotor2.setTargetPosition(0);

                RV4BMotor1.setPower(MACRO_POWER);
                RV4BMotor2.setPower(MACRO_POWER);
            }

            if (gamepad1.left_trigger >= 0.4 && RV4BMotor1.getCurrentPosition() < 0 && RV4BMotor2.getCurrentPosition() < 0){ //go down manually
                RV4BMotor1.setTargetPosition(RV4BMotor1.getCurrentPosition() + ARM_SPEED_MANUAL);
                RV4BMotor2.setTargetPosition(RV4BMotor2.getCurrentPosition() + ARM_SPEED_MANUAL);

                RV4BMotor1.setPower(ARM_POWER);
                RV4BMotor2.setPower(ARM_POWER);
            }
            else if (gamepad1.right_trigger >= 0.4 && RV4BMotor1.getCurrentPosition() > -690 && RV4BMotor2.getCurrentPosition() > -690){ //go up manually
                RV4BMotor1.setTargetPosition(RV4BMotor1.getCurrentPosition() - ARM_SPEED_MANUAL);
                RV4BMotor2.setTargetPosition(RV4BMotor2.getCurrentPosition() - ARM_SPEED_MANUAL);

                RV4BMotor1.setPower(ARM_POWER);
                RV4BMotor2.setPower(ARM_POWER);
            }

            //If the values want to slant, fix it!
            int diffTarget = RV4BMotor1.getTargetPosition() - RV4BMotor2.getTargetPosition();
            RV4BMotor1.setTargetPosition(RV4BMotor1.getTargetPosition() - diffTarget/2);
            RV4BMotor2.setTargetPosition(RV4BMotor2.getTargetPosition() + diffTarget/2);

            //If the values are slanting, adjust for it!
            int diff = RV4BMotor1.getCurrentPosition() - RV4BMotor2.getCurrentPosition();
            if (RV4BMotor1.getTargetPosition() > RV4BMotor1.getCurrentPosition()) { //If we're trying to increase...
                RV4BMotor1.setPower(RV4BMotor1.getPower() * (1 - diff / ARM_SPEED_MANUAL));
                RV4BMotor2.setPower(RV4BMotor1.getPower() * (1 + diff / ARM_SPEED_MANUAL));
            }
            else{ //If we're trying to decrease...
                RV4BMotor1.setPower(RV4BMotor1.getPower() * (1 + diff / ARM_SPEED_MANUAL));
                RV4BMotor2.setPower(RV4BMotor1.getPower() * (1 - diff / ARM_SPEED_MANUAL));
            }

            //Handle claw open and close
            if (gamepad1.left_bumper){
                clawFinger.setPosition(0.1); //close
                fingerPos = "Closed";
            }
            else if (gamepad1.right_bumper){
                clawFinger.setPosition(0.4); //open
                fingerPos = "Open";
            }

            //Drive!
            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;
            // Normalize the values so neither exceed +/- 1.0
            left = Math.tanh(left);
            right = Math.tanh(right);
            
            //90 degree buttons
            if(gamepad1.dpad_left){
                degrees = orientation.getYaw(AngleUnit.DEGREES) - 90;
                turning90 = true;
            }
            else if(gamepad1.dpad_right){
                degrees = orientation.getYaw(AngleUnit.DEGREES) + 90;
                turning90 = true;
            }
            if (turning90){
                orientation = imu.getRobotYawPitchRollAngles();
                double prevAngle = currentAngle;
                currentAngle = orientation.getYaw(AngleUnit.DEGREES);
                
                errorTurn = degrees - currentAngle;

                //get most efficient angle (imu has angles from -180 to 180)
                if (errorTurn > 180) {
                    errorTurn -= 360;
                } else if (errorTurn < -180) {
                    errorTurn += 360;
                }

                double turnP = K_P_TURN * errorTurn * 5.969; //convert angle to ticks so that the P still applies
                double turnD = D_MULT_TURN * (currentAngle - prevAngle);

                double powerTurn = Math.tanh(turnP + turnD); //Normalize power to +/- 1.0

                telemetry.addLine("ROTATING");

                // Average left and right with powerTurn to retain normalization
                leftDrive.setPower ((left - powerTurn) * 0.5 * SPEED_MULT);
                rightDrive.setPower((right + powerTurn) * 0.5 * SPEED_MULT);
            }
            else{
                // Output the normalized vales to the motor drives.
                leftDrive.setPower(left * SPEED_MULT);
                rightDrive.setPower(right * SPEED_MULT);
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("Speed", speed);
            telemetry.addData("Stick X",  "%.2f", turn);
            telemetry.addData("Stick Y", "%.2f", (drive * -1));
            telemetry.addData("Claw", fingerPos);
            telemetry.addData("RV4B Power A", "%.2f", RV4BMotor1.getPower());
            telemetry.addData("RV4B Power B", "%.2f", RV4BMotor2.getPower());
            telemetry.addData("RV4B Motor A Encoder", RV4BMotor1.getCurrentPosition());
            telemetry.addData("RV4B Motor B Encoder", RV4BMotor2.getCurrentPosition());
            telemetry.addData("RV4B Motor A Encoder Target", RV4BMotor1.getTargetPosition());
            telemetry.addData("RV4B Motor B Encoder Target", RV4BMotor1.getTargetPosition());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(35);
        }
    }
}
