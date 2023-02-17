/* Copyright (c) 2022 FIRST. All rights reserved.
 *
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

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import android.util.Log;

//import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.core.subsystems.Eye;
//import org.firstinspires.ftc.teamcode.core.subsystems.EyeAll;
//import org.firstinspires.ftc.teamcode.util.AllianceConfig;
//import org.firstinspires.ftc.teamcode.util.Constants;
//import org.firstinspires.ftc.teamcode.util.HandMotorsPosition;
//import org.firstinspires.ftc.teamcode.util.PID;
//import org.firstinspires.ftc.teamcode.util.Util;

import java.util.Locale;
import java.util.Objects;

/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Game Robot Autonomous", group="Robot") // Drive
//@Disabled
public class RobotAutonomousDrive extends OpMode
{
    /* Declare OpMode members. */
    //private DcMotor frontLeft;
    //private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    //private BNO055IMU imu;
    private IMU imu         = null;      // Control/Expansion Hub IMU
    
    private DcMotor lifterMotor;
    //private DcMotor rotatorMotor;
    //private DcMotor armMotor;
    
    //private Servo eyeServo;
    //private Servo elbowServo;
    //private Servo wristServo;
    //private Servo palmServo;
    private Servo fingerServo;
    
    //EyeAll eye;

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int backLeftTarget = 0;
    private int backRightTarget = 0;
    //private int frontLeftTarget = 0;
    //private int frontRightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 20 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    
    private final ElapsedTime autoTimer30Sec = new ElapsedTime();
    
    // Eye needs to access it, public!
    //public static AllianceConfig.AllianceConfigData allianceConfig =
    //        new AllianceConfig.AllianceConfigData();
    
    //HandMotorsPosition PredefinedPosition = new HandMotorsPosition();
    
    /**
     * Initialize any subsystems that need initializing before the game starts.
     */
    @Override
    public void init()
    {
        autoTimer30Sec.reset();
        
        //eye = new EyeAll(hardwareMap);
        
        //eye.autoInit();
    
        // Initialize the drive system variables.
        //frontLeft = hardwareMap.get(DcMotor.class, Constants.leftfrontMotor);
        //frontRight = hardwareMap.get(DcMotor.class, Constants.rightfrontMotor);
        backLeft = hardwareMap.get(DcMotor.class, "MotorA");
        backRight = hardwareMap.get(DcMotor.class, "MotorB");
    
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

    
        // define initialization values for IMU, and then initialize it.
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        //imu = hardwareMap.get(BNO055IMU.class, Constants.imu);
        //imu.initialize(parameters);
        imu = hardwareMap.get(IMU.class, "imu");
        // define initialization values for IMU, and then initialize it.
        // The next three lines define the desired axis rotations.
        // To Do: EDIT these values to match YOUR mounting configuration.
        double xRotation = 0;  // enter the desired X rotation angle here.
        double yRotation = 0;  // enter the desired Y rotation angle here.
        double zRotation = 0;  // enter the desired Z rotation angle here.
    
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // Reset Yaw
        imu.resetYaw();
        
    
        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
        // Set the encoders for closed loop speed control, and reset the heading.
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        resetHeading();
    
        fingerServo = hardwareMap.get(Servo.class, "Servo1");
    
        lifterMotor = hardwareMap.get(DcMotor.class, "MotorC");

        // reset all motors encoder to zero. remove them since we use saved cali data
        //lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        lifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //AllianceConfig.ReadConfigFromFile(allianceConfig);
        //allianceConfig.ParkingZone = "-1";
        
        //PredefinedPosition.Load();
    
        fingerServo.setPosition(0.2);
    
        targetPositionLifter = lifterMotor.getCurrentPosition();
    
        //eye.OpenEyeToRead();
    }
    
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    // Tag ID 121, 122, 123 from the 36h11 family
    int ZONE_1 = 121;
    int ZONE_2 = 122;
    int ZONE_3 = 123;
    
    //int AprilTagDetectionId = -1;
    
    @Override
    public void init_loop()
    {
        /*if(eye.ReadParkingID() == ZONE_1 )
        {
            allianceConfig.ParkingZone = "1";
        }
        else if (eye.ReadParkingID() == ZONE_2 )
        {
            allianceConfig.ParkingZone = "2";
        }
        else if(eye.ReadParkingID() == ZONE_3)
        {
            allianceConfig.ParkingZone = "3";
        }*/
    
        TuningHandMotors(gamepad1); // TODO debug
        lifterMotorRunnable();
        
        /*telemetry.addData("TEAM #: ",
                allianceConfig.TeamNumber);
        telemetry.addData("ALLIANCE #: ",
                allianceConfig.Alliance);
        telemetry.addData("STARTING LOCATION #: ",
                allianceConfig.Location);
        telemetry.addData("PARKING ZONE #:",
                allianceConfig.ParkingZone);*/
    
        composeTelemetry();
    
        telemetry.update();
    }
    
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        missionRunTimer.reset();
    }
    
    // SPOT is loop-able function, MOVE is onetime execution function
    private enum Mission
    {
        SPOT_A, // init position
        MOVE_A2B,
        SPOT_B, // Mid Junction, face to 0 degree
        MOVE_B2C,
        SPOT_C, // Cones stock
        MOVE_C2D,
        SPOT_D, // Mid Junction, face to -90 degree
        MOVE_D2C,
        MOVE_C2E,
        MOVE_D2E,
        SPOT_E, // parking zone, final stop
        EXIT
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    
    int missionStepTimeout = 10; // TODO debug
    @Override
    public void loop()
    {
        switch (currentMission)
        {
            case SPOT_A:
                // find self init position, read parking zone picture, drop preload to low junction
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                spotA();
                break;
            /*case SPOT_B:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                spotB(-520, false);
                break;
            case SPOT_C:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                // pick up a new cone from stock
                spotC();
                break;
            case SPOT_D:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                // drop cone on mid junction
                spotB(-500, true);
                break;
            case SPOT_E:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                // final parking zone
                break;*/
            case MOVE_A2B:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                moveA2B();
                break;
            /*case MOVE_B2C:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                moveB2C();
                break;
            case MOVE_C2D:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                moveC2D();
                break;
            case MOVE_D2C:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                moveD2C();
            case MOVE_C2E:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                moveC2E();
                break;
            case MOVE_D2E:
                if (missionRunTimer.seconds() > missionStepTimeout)
                {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                moveD2E();
                break;*/
            default: //EXIT
                break;
        }
        
        lifterMotorRunnable();
    
        //addLog();
    
        TuningHandMotors(gamepad1); // TODO debug
    
        //sendTelemetry(true);
        composeTelemetry(); //don't need it, just update it with presets
        
        telemetry.update();
        //sleep(1000);  // Pause to display last telemetry message.
    }
    
    //yeAll.ObjectLocation isCenterPole = EyeAll.ObjectLocation.UNKNOWN;
    
    
    private Mission currentMission = Mission.SPOT_A; // debug: TODO
    private Mission previousMission = Mission.SPOT_A;
    ElapsedTime missionRunTimer = new ElapsedTime();
    private void setMissionTo(Mission newMission)
    {
        // debug: TODO
        /*if(newMission == Mission.MOVE_A2B)
        {
            currentMission = Mission.EXIT;
            missionStepTimeout = 1000;
            return;
        }*/
        // end debug
    
        previousMission = currentMission;
        currentMission = newMission;
        missionRunTimer.reset();
        setTaskTo(0);
    }
    private int currentTaskID = 0;
    private int previousTaskID = 0;
    private final ElapsedTime taskRunTimeout = new ElapsedTime();
    private void setTaskTo(int newTaskID)
    {
        previousTaskID = currentTaskID;
        currentTaskID = newTaskID;
        taskRunTimeout.reset();
    }
    
    private void spotA()
    {
        if (currentTaskID == 0)
        {
            moveA2B_Strafe0 = 27;
            moveA2B_Straight0 = 25.5;
            moveB2C_Straight01 = 27.;
            moveB2C_Straight02 = 42.;
            moveB2C_Heading02 = -90.0;
            moveC2D_Straight01 = -39.;
            moveC2D_Heading01 = -90.;
            moveD2C_Straight01 = 32.;
            moveD2C_Heading01 = -90.;
            //moveD2E_Straight01 = 34.;
            moveD2E_Heading01 = -90.;
            //moveC2E_Straight01 = 34.;
            moveC2E_Heading01 = -90.;
    
            /*if (Objects.equals(allianceConfig.ParkingZone, "1")) moveD2E_Straight01 = 30;
            else if (Objects.equals(allianceConfig.ParkingZone, "2")) moveD2E_Straight01 = 20;
            else moveD2E_Straight01 = 0;
    
            if (Objects.equals(allianceConfig.ParkingZone, "1")) moveC2E_Straight01 = -30;
            else if (Objects.equals(allianceConfig.ParkingZone, "2")) moveC2E_Straight01 = -20;
            else moveC2E_Straight01 = 0;*/
            
    
            targetPositionArm = 1500;
            
            setTaskTo(1);
        }
        else if (currentTaskID == 1)
        {
            if (taskRunTimeout.milliseconds() > 3000)
            {
                setTaskTo(2);
            }
        }
        /*else if(currentTaskID == 2)
        {
            if(!Objects.equals(allianceConfig.ParkingZone, "0"))
            {
                setTaskTo(3);
            }
            else // "0" so far
            {
                if(taskRunTimeout.seconds() < 2)
                {
                    allianceConfig.ParkingZone = String.valueOf(eye.ReadParkingID());
                }
                else // timeout
                {
                    setTaskTo(3);
                }
            }
        }*/
        else // loopTaskCount =
        {
            
            setMissionTo(Mission.MOVE_A2B);
        }
    }
    
    double moveA2B_Strafe0;
    double moveA2B_Straight0;
    private void moveA2B()
    {
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        
        if (currentTaskID == 0)
        {
            setTaskTo(1);
        }
        else if (currentTaskID == 1)
        {
            boolean done = turnToHeadingLoop(0.3, -90);
            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setTaskTo(2);
            }
        }
        else if (currentTaskID == 2)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, moveA2B_Straight0, -90);
            if (taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (done)
            {
                setMissionTo(Mission.EXIT);// todo debug
                //setMissionTo(Mission.SPOT_B);
            }
            //targetPositionArm = 1000; // rising arm up while moving forward
        }
    }
    int error_lr;
    int error_fn;
    double chassisMoveDistance = 0.;

    
    double moveB2C_Straight01;
    double moveB2C_Straight02;
    double moveB2C_Heading02;
    
    double extraDistance = 0.;

    


    
    double moveC2D_Straight01;
    double moveC2D_Heading01;

    
    double moveD2C_Straight01;
    double moveD2C_Heading01;

    
    double moveD2E_Straight01;
    double moveD2E_Heading01;

    
    double moveC2E_Straight01;
    double moveC2E_Heading01;

    
    int targetPositionLifter;
    int targetPositionArm;
    private final PID armPID = new PID(0.0035,0,0.025,0.1, 0.5, -0.2);
    

    

    

    
    private void lifterMotorRunnable()
    {
        //lifterMotor.setPower(0);
    }
    
    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    public void resetDriveLoops()
    {
        hasInitStraight = false;
        hasInitTurnTo = false;
        hasInitHold = false;
    }
    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    *  maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    *  distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    *  heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    private boolean hasInitStraight = false;
    public boolean driveStraightLoop(double maxDriveSpeed,
                                     double distance,
                                     double heading)
    {
        if(!hasInitStraight)
        {
            hasInitStraight = true;
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            backLeftTarget = backLeft.getCurrentPosition() + moveCounts;
            backRightTarget = backRight.getCurrentPosition() + moveCounts;
            //frontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
            //frontRightTarget = frontRight.getCurrentPosition() + moveCounts;
        
            // Set Target FIRST, then turn on RUN_TO_POSITION
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);
            //frontLeft.setTargetPosition(frontLeftTarget);
            //frontRight.setTargetPosition(frontRightTarget);
        
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0, false);
        }
        
        // keep looping while we are still active, and all motors are running.
        if (backLeft.isBusy() && backRight.isBusy() )
        {
        
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
        
            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
            {
                turnSpeed *= -1.0;
    
                // Apply the turning correction to the current driving speed.
                moveRobot(maxDriveSpeed, turnSpeed, true);
            }
            else
            {
                moveRobot(maxDriveSpeed, turnSpeed, false);
            }
        
            // Display drive status for the driver.
            //sendTelemetry(true);
            return false;
        }
        else
        {
            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, false);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetDriveLoops();
            return true;
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     *  maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     *  heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    private boolean hasInitTurnTo = false;
    public boolean turnToHeadingLoop(double maxTurnSpeed, double heading)
    {
        if(!hasInitTurnTo)
        {
            hasInitTurnTo = true;
            // Run getSteeringCorrection() once to pre-calculate the current error
            getSteeringCorrection(heading, P_DRIVE_GAIN);
        }
    
        // keep looping while we are still active, and not on heading.
        if (Math.abs(headingError) > HEADING_THRESHOLD)
        {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed,false);

            // Display drive status for the driver.
            //sendTelemetry(false);
            return false;
        }
        else
        {
            // Stop all motion;
            moveRobot(0, 0,false);
            resetDriveLoops();
            return true;
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     *  maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     *  heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     *  holdTime   Length of time (in seconds) to hold the specified heading.
     */
    private boolean hasInitHold = false;
    ElapsedTime holdTimer = new ElapsedTime();
    public boolean holdHeadingLoop(double maxTurnSpeed, double heading, double holdTime)
    {
        if(!hasInitHold)
        {
            hasInitHold = true;
            holdTimer.reset();
        }
        
        if (holdTimer.time() < holdTime)
        {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed, false);

            // Display drive status for the driver.
            //sendTelemetry(false);
            return false;
        }
        else
        {
            // Stop all motion;
            moveRobot(0, 0, false);
            resetDriveLoops();
            return true;
        }
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    private double getSteeringCorrection(double desiredHeading, double proportionalGain)
    {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    private void moveRobot(double drive, double turn, boolean isBackward)
    {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        /*if(isBackward)
        {
            leftSpeed = -1*leftSpeed;
            rightSpeed = -1*rightSpeed;
        }*/
        
        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);
    }
    
    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    private double getRawHeading()
    {
        //Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return angles.firstAngle;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Reset the "offset" heading back to zero
     */
    private void resetHeading()
    {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
    
    // ****************  updating telemetry **********************
    //Orientation currentRobotAngles;
    YawPitchRollAngles currentRobotAngles;
    private void composeTelemetry()
    {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            currentRobotAngles = imu.getRobotYawPitchRollAngles();
            //currentRobotAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });
        
        /*telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });*/
    
        telemetry.addLine()
                .addData("Mission #", currentMission)
                .addData("Task #", currentTaskID);
    
        //telemetry.addData("Camera On?", eye.IsOpen);
        //telemetry.addData("Pole Location", isCenterPole);
        //telemetry.addData("Cone Location", isConeCenter);
    
        telemetry.addData("IMU heading", "%.2f", getRawHeading());
    
        telemetry.addLine()
                .addData("Finger", "%.2f", fingerServo.getPosition());
    
        telemetry.addLine().addData("Lifter", lifterMotor.getCurrentPosition());
    }
    
    private void sendTelemetry(boolean straight)
    {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos bL:bR",  "%7d:%7d",
                    backLeftTarget, backRightTarget);
        } else {
            telemetry.addData("Motion", "Turning");
        }
        
        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.addData("Wheel Positions L:R",  "%7d:%7d",      backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());
    }
    
    private String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    
    private String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    
    private void addLog()
    {
        String a = String.format(
                new String("Lm(%d : %d : %.2f) Rm(%d : %d : %.2f) Am(%d : %d : %.2f) " +
                        "IMU(%.2f) DR(%.2f:%.2f:%.2f:%.2f:%.2f:%.2f) Vi(%s:%d; P:%s/C:%s; LR:%d,FN:%d)"),
                lifterMotor.getCurrentPosition(),
                lifterMotor.getTargetPosition(),
                lifterMotor.getPower(),
                
                getRawHeading(),

                backLeft.getPower(),
                backRight.getPower(),
                driveSpeed,
                turnSpeed,
        
                currentMission.toString(),
                currentTaskID,
                //isCenterPole.toString(),
                //isConeCenter.toString(),
                error_lr,
                error_fn
        );
        Log.println(Log.INFO,"Status", a);
    }
    
    final double TRIGGER_THRESHOLD = 0.75;     // Squeeze more than 3/4 to get rumble.
    private boolean eyeServoRightLock = false;
    private boolean eyeServoLeftLock = false;
    //private boolean wristServoRightLock = false;
    //private boolean wristServoLeftLock = false;
    private boolean palmServoLeftLock = false;
    private boolean palmServoRightLock = false;
    private boolean lifterHighLock = false;
    private boolean lifterLowLock = false;
    //private boolean leftTriggerLock = false;
    //private boolean armPositionLock = false;
    ElapsedTime runtimeManual = new ElapsedTime();
    
    private void TuningHandMotors(Gamepad gamepad)
    {
        
        // finger servo
        if (gamepad.y)
        {
            fingerServo.setPosition(1.);//open
        }
        if (gamepad.a)
        {
            fingerServo.setPosition(0.1);// close
        }
        
        // arm motor
        /*double updown = -1 * Util.applyDeadband(gamepad.left_stick_y, 0.1);
        if( updown > 0)
            updown = updown * 60;
        else
            updown = updown * 40;
        targetPositionArm += updown;*/
        if(gamepad.dpad_up)
        {
            targetPositionArm += 10;
        }
        else if(gamepad.dpad_down)
        {
            targetPositionArm -= 10;
        }
        
        //lifter motor
        if (gamepad.left_stick_button)
        {//up
            if (!lifterHighLock)
            {
                lifterHighLock = true;  // Hold off any more triggers
                if (lifterMotor.getCurrentPosition() < 2000)
                { // max high limit
                    lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition() + 100);
                    lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtimeManual.reset();
                    lifterMotor.setPower(0.2);
                    while ((runtimeManual.seconds() < 1) &&
                            (lifterMotor.isBusy()))
                    {
                    }
                }
                //gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
            }
        }
        else
        {
            lifterHighLock = false;  // We can trigger again now.
        }
        
        if (gamepad.right_stick_button)
        {//down
            if (!lifterLowLock)
            {
                lifterLowLock = true;
                if (lifterMotor.getCurrentPosition() > 100) // min low limit
                {
                    lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition() - 100);
                    lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtimeManual.reset();
                    lifterMotor.setPower(0.2);
                    while ((runtimeManual.seconds() < 1) &&
                            (lifterMotor.isBusy()))
                    {
                    }
                }
            }
        }
        else
        {
            lifterLowLock = false;
        }
    }
}
