/* Copyright (c) 2021 FIRST. All rights reserved.
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.ArrayList;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


@TeleOp(name="OmniWheels 0.2", group="OmniOp")
@Disabled
public class OmniWheels extends LinearOpMode {
    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    private DcMotor leftFrontWheel = null; //Motors to control all wheels
    private DcMotor leftBackWheel = null;
    private DcMotor rightFrontWheel = null;
    private DcMotor rightBackWheel = null;
    
    private DcMotor topArmBaseJoint = null; //Stronger joints for the arms
    private DcMotor topArmMiddleJoint = null;
    
    private DcMotor leftLinearActuator = null; //Linear Actuators to lift platform need to be strong, liekly will be slow
    private DcMotor rightLinearActuator = null;
    
    private Servo topHand = null; //Hands for the arms
    private Servo bottomHand = null;
    
    private Servo bottomArmBaseJoint = null; //One more joint, just its a servo

    private double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    private boolean rotationDirection = true; //clockwise or counterclockwise

    List<DcMotor> allMotors = new ArrayList<>();
    List<Servo>   allServos = new ArrayList<>();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    @Override
    public void runOpMode() {
 
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontWheel  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackWheel  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontWheel = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackWheel = hardwareMap.get(DcMotor.class, "right_back_drive");
        
        topArmBaseJoint = hardwareMap.get(DcMotor.class, "top_arm_base_joint");
        topArmMiddleJoint = hardwareMap.get(DcMotor.class, "top_arm_middle_joint");
        
        leftLinearActuator = hardwareMap.get(DcMotor.class, "left_linear_actuator");
        rightLinearActuator = hardwareMap.get(DcMotor.class, "right_linear_actuator");

        allMotors.add(leftFrontWheel);
        allMotors.add(leftBackWheel);
        allMotors.add(rightFrontWheel);
        allMotors.add(rightBackWheel);
        allMotors.add(topArmBaseJoint);
        allMotors.add(topArmMiddleJoint);
        allMotors.add(leftLinearActuator);
        allMotors.add(rightLinearActuator);
        
        topHand = hardwareMap.get(Servo.class, "top_hand");
        bottomHand = hardwareMap.get(Servo.class, "bottom_hand");
        
        bottomArmBaseJoint = hardwareMap.get(Servo.class, "bottom_arm_base_joint");

        allServos.add(topHand);
        allServos.add(bottomHand);
        allServos.add(bottomBaseJoint);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotor.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        rightBackWheel.setDirection(DcMotor.Direction.FORWARD);
        
        topArmBaseJoint.setDirection(DcMotor.Direction.FORWARD);
        topArmMiddleJoint.setDirection(DcMotor.Direction.FORWARD);
        leftLinearActuator.setDirection(DcMotor.Direction.FORWARD);
        rightLinearActuator.setDirection(DcMotor.Direction.FORWARD);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        
        // Wait for the game to start (driver presses START)
        telemetry.addData("High Five", "We Roboted! Woohoo!!!");
        telemetry.update();

        waitForStart();
        runtime.reset();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Axial", axial();
            telemetry.addData("Lateral", lateral();
            telemetry.addData("Yam", yam();
            
            double max;
            double armMax;
            final double INCREMENT     = 0.01;   // amount to slew servo each CYCLE_MS cycle
            final int    CYCLE_MS      = 50;     // period of each cycle
            final double MAX_POS       = 1.0;    // Maximum rotational position
            final double MIN_POS       = 0.0;    // Minimum rotational position
            // All variable numbers from here is guestimations, needs to be actually tested ==============================================================================
            final double BOTTOM_BASE_0 = 0.0;    // Furthest back the bottom arm base needs to go
            final double TOP_BASE_0    = 0.0;    // Furthest back the top arm base needs to go
            final double TOP_MIDDLE_0  = 0.0;    // Furthest back the top arm middle needs to go
            final double ALL_HANDS_0   = 0.0;    // Close position for both hands 
            final doubale OPEN_TOPHAND = 1.0;    // Open the top hand fully
            final double CLOSE_TOPHAND = 0.5;    // Close the top hand fully

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value - Forward and Backward 
            double lateral =  gamepad1.right_stick_x;  // Strafe - Left and Right 
            double yaw     =  gamepad1.left_stick_x;  // Rotation - ClockWise and CounterClockWise 
            
            // On both controllers
            boolean pos0GP1      = gamepad1.b; // Set all robot to wanted 0 position if not initally in wanted spot 
            boolean linearUpGP1   = gamepad1.left_bumper; // Move linear actuators to go up 
            boolean linearDownGP1 = gamepad1.right_bumper; // Move linear actuators to go down 
            
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            // boolean grab = gamepad2.left_stick_x;
            boolean topArmPresetGrab       = gamepad2.y; // Grab sample from bottom arm
            boolean topArmPresetRelease    = gamepad2.a; // Put top arm in wanted spot to let go and let go
            boolean bottomArmPresetGrab    = gamepad2.dpad_down; // go out and grab sample
            boolean bottomArmPresetRelease = gamepad2.dpad_up; // go in and get in spot for top arm to grab sample

            double topArmBase       = gamepad2.left_stick_y;      // Move the Base Joint Forward/Backward
            double topArmMiddle     = gamepad2.left_stick_y;     // Move the Middle Joint Forward/Backward
            double topArmPower      = topArmBase - topArmMiddle;
            
            // On both controllers
            boolean pos0GP2      = gamepad2.b; // Set all robot to wanted 0 position if not initally in wanted spot 
            boolean linearUpGP2   = gamepad2.left_bumper; // Move linear actuators to go up 
            boolean linearDownGP2 = gamepad2.right_bumper; // Move linear actuators to go down 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            // Acting as forward, turn right, and strafe right, is positive, will likely need to change.
            // Both Left wheels are reversed
            double leftFrontPower  = -axial - lateral - yaw;
            double rightFrontPower = -axial - lateral + yaw;
            double leftBackPower   = -axial + lateral - yaw;
            double rightBackPower  = -axial + lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            
            armMax = Math.max(Math.abs(topArmPower));

            
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            if (armMax > 1.0) {
                topArmPower /= armMax;
            }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            if (pos0GP1 == true || pos0GP2 == true) {
                topHand.setPosition(ALL_HANDS_0);
                bottomHand.setPosition(ALL_HANDS_0);
                bottomArmBaseJoint.setPosition(BOTTOM_BASE_0);
            }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            if (linearUpGP1 == true || linearUpGP2 == true) {
                // Set the motor position, not measuring time
            } else if (linearDownGP1 == true || linearDownGP2 == true) {
                // Set the motor position, not measuring time
            }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            if (topArmPresetGrab) {
                topArmBaseJoint.setTargetPosition(1 * 1440); // change 1 to be the correct number
                topArmMiddleJoint.setTargetPosition(1 * 1440); // change 1 to be the correct number
                topHand.setPosition(CLOSE_TOPHAND); // Close the top hand on sample
            } else if (topArmPresetRelease) {
                topArmBaseJoint.setTargetPosition(1 * 1440); // change 1 to be the correct number
                topArmMiddleJoint.setTargetPosition(1 * 1440); // change 1 to be the correct number
                topHand.setPosition(OPEN_TOPHAND);
            } else if (bottomArmPresetGrab) {
                bottomArmBaseJoint.setTargetPosition(1 * 1440); // change 1 to be the correct number
                bottomArmMiddleJoint.setTargetPosition(1 * 1440); // change 1 to be the correct number
                bottomHand.setPosition(CLOSE_TOPHAND);
            } else if (bottomArmPresetRelease) {
                bottomArmBaseJoint.setTargetPosition(1 * 1440); // change 1 to be the correct number
                bottomArmMiddleJoint.setTargetPosition(1 * 1440); // change 1 to be the correct number
                bottomHand.setPosition(OPEN_TOPHAND);
            }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            // MAKE MANUAL CONTROLS FOR TOP ARM HERE =====================================================================================================================================
            if (_Up) { // Still need to define the power var
                topArmBaseJoint.setTargetPosition(-1 * 1440); // change 1 to be the correct number
                topArmMiddleJoint.setTargetPosition(-1 * 1440); // change 1 to be the correct number
            } else if (_Down) { // Still need to define the power var
                topArmBaseJoint.setTargetPosition(1 * 1440); // change 1 to be the correct number
                topArmMiddleJoint.setPosition(.5 * 1440); // change 1 to be the correct number
            }
            
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontWheel.setPower(leftFrontPower);
            rightFrontWheel.setPower(rightFrontPower);
            leftBackWheel.setPower(leftBackPower);
            rightBackWheel.setPower(rightBackPower);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
            // Show the elapsed game time and wheel power.
            // telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            // telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            for (DcMotor thisMotor in allMotors) {
                telemetry.addData("MotorSpeed", thisMotor.getSpeed());
            }
            for (Servo thisServo in allServos) {
                telemetry.addData("ServoPosition", thisServo.getPosition());
            }
            telemetry.update();
        }
    }}
