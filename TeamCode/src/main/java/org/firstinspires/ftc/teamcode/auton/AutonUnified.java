package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.content.SharedPreferences;

import org.firstinspires.ftc.teamcode.auton.AutonMovement;
import org.firstinspires.ftc.teamcode.logic.CommonDefs;
import org.firstinspires.ftc.teamcode.logic.RobotState;
import com.pedropathing.geometry.Pose;

/**
 * Unified Autonomous OpMode with menu selection
 * Allows selection of Alliance, Position, and Row Count via gamepad
 */
@Autonomous(name = "AutonUnified", group = "Autonomous")
public class AutonUnified extends LinearOpMode {
    
    // Menu state
    private CommonDefs.Alliance selectedAlliance = null;
    private CommonDefs.PositionType selectedPosition = null;
    private CommonDefs.AutonRowsToCollect selectedRows = null;
    
    // Hierarchical menu navigation
    private int currentStep = 0; // 0=Alliance, 1=Position, 2=Rows, 3=Complete
    private final int TOTAL_STEPS = 3;
    private final String[] STEP_NAMES = {"Alliance", "Position", "Rows"};
    
    // Current options for each step
    private final CommonDefs.Alliance[] ALLIANCE_OPTIONS = {CommonDefs.Alliance.BLUE, CommonDefs.Alliance.RED};
    private final CommonDefs.PositionType[] POSITION_OPTIONS = {CommonDefs.PositionType.CLOSE, CommonDefs.PositionType.FAR};
    private final CommonDefs.AutonRowsToCollect[] ROWS_OPTIONS = {CommonDefs.AutonRowsToCollect.ROS_1, CommonDefs.AutonRowsToCollect.ROS_2, CommonDefs.AutonRowsToCollect.ROS_3};
    
    // Current selection index within each step
    private int allianceIndex = 0;
    private int positionIndex = 0;
    private int rowsIndex = 0;
    
    @Override
    public void runOpMode() {
        // Initialize telemetry
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        
        // Show menu until all selections are made and START is pressed
        while (!isStarted() && !isStopRequested()) {
            handleMenuInput();
            displayMenu();
            sleep(100); // Prevent button spam
        }
        
        // Wait for start
        waitForStart();
        
        if (opModeIsActive() && currentStep == 3 && selectedAlliance != null && selectedPosition != null && selectedRows != null) {
            // Create and run autonomous sequence with selected parameters
            AutonMovement autonMovement = new AutonMovement(this, selectedAlliance, selectedPosition, selectedRows);
            autonMovement.runAutonomousSequence();
            
            // Save final pose to SharedPreferences for TeleOp
            savePoseToSharedPreferences();
        } else if (opModeIsActive() && currentStep < 3) {
            // If START was pressed before completing all selections
            telemetry.addData("Error", "Complete all selections before starting!");
            telemetry.update();
            sleep(2000);
        }
    }
    
    private void handleMenuInput() {
        // Navigate options with dpad up/down
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            switch (currentStep) {
                case 0: // Alliance selection
                    allianceIndex = (allianceIndex - 1 + ALLIANCE_OPTIONS.length) % ALLIANCE_OPTIONS.length;
                    break;
                case 1: // Position selection
                    positionIndex = (positionIndex - 1 + POSITION_OPTIONS.length) % POSITION_OPTIONS.length;
                    break;
                case 2: // Rows selection
                    rowsIndex = (rowsIndex - 1 + ROWS_OPTIONS.length) % ROWS_OPTIONS.length;
                    break;
            }
            sleep(200); // Debounce
        }
        
        if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            switch (currentStep) {
                case 0: // Alliance selection
                    allianceIndex = (allianceIndex + 1) % ALLIANCE_OPTIONS.length;
                    break;
                case 1: // Position selection
                    positionIndex = (positionIndex + 1) % POSITION_OPTIONS.length;
                    break;
                case 2: // Rows selection
                    rowsIndex = (rowsIndex + 1) % ROWS_OPTIONS.length;
                    break;
            }
            sleep(200); // Debounce
        }
        
        // Confirm selection and move to next step with A button or dpad_right
        if (gamepad1.a || gamepad1.dpad_right) {
            switch (currentStep) {
                case 0: // Confirm Alliance
                    selectedAlliance = ALLIANCE_OPTIONS[allianceIndex];
                    currentStep = 1;
                    break;
                case 1: // Confirm Position
                    selectedPosition = POSITION_OPTIONS[positionIndex];
                    currentStep = 2;
                    break;
                case 2: // Confirm Rows
                    selectedRows = ROWS_OPTIONS[rowsIndex];
                    currentStep = 3; // All selections complete
                    break;
            }
            sleep(300); // Debounce
        }
        
        // Go back to previous step with B button or dpad_left
        if (gamepad1.b || gamepad1.dpad_left) {
            if (currentStep > 0) {
                currentStep--;
                // Clear the selection for the step we're going back to
                switch (currentStep) {
                    case 0:
                        selectedAlliance = null;
                        break;
                    case 1:
                        selectedPosition = null;
                        break;
                    case 2:
                        selectedRows = null;
                        break;
                }
            }
            sleep(300); // Debounce
        }
    }
    
    private void displayMenu() {
        telemetry.clear();
        telemetry.addData("=== AUTONOMOUS SETUP ===", "");
        
        // Show current step progress
        telemetry.addData("Step", String.format("%d of %d: %s", 
                         currentStep + 1, TOTAL_STEPS, 
                         currentStep < TOTAL_STEPS ? STEP_NAMES[currentStep] : "Complete"));
        telemetry.addData("", "");
        
        // Show current selections made so far
        telemetry.addData("--- CURRENT SELECTIONS ---", "");
        telemetry.addData("Alliance", selectedAlliance != null ? 
                         (selectedAlliance == CommonDefs.Alliance.BLUE ? "BLUE" : "RED") : "Not selected");
        telemetry.addData("Position", selectedPosition != null ? 
                         (selectedPosition == CommonDefs.PositionType.CLOSE ? "CLOSE" : "FAR") : "Not selected");
        telemetry.addData("Rows", selectedRows != null ? 
                         getRowsDisplayText(selectedRows) : "Not selected");
        telemetry.addData("", "");
        
        // Show current step selection
        if (currentStep < TOTAL_STEPS) {
            telemetry.addData("--- CHOOSE " + STEP_NAMES[currentStep].toUpperCase() + " ---", "");
            
            switch (currentStep) {
                case 0: // Alliance selection
                    for (int i = 0; i < ALLIANCE_OPTIONS.length; i++) {
                        String indicator = (i == allianceIndex) ? ">>> " : "    ";
                        String value = (ALLIANCE_OPTIONS[i] == CommonDefs.Alliance.BLUE) ? "BLUE" : "RED";
                        telemetry.addData(indicator, value);
                    }
                    break;
                case 1: // Position selection
                    for (int i = 0; i < POSITION_OPTIONS.length; i++) {
                        String indicator = (i == positionIndex) ? ">>> " : "    ";
                        String value = (POSITION_OPTIONS[i] == CommonDefs.PositionType.CLOSE) ? "CLOSE" : "FAR";
                        telemetry.addData(indicator, value);
                    }
                    break;
                case 2: // Rows selection
                    for (int i = 0; i < ROWS_OPTIONS.length; i++) {
                        String indicator = (i == rowsIndex) ? ">>> " : "    ";
                        String value = getRowsDisplayText(ROWS_OPTIONS[i]);
                        telemetry.addData(indicator, value);
                    }
                    break;
            }
            
            telemetry.addData("", "");
            telemetry.addData("Controls:", "UP/DOWN: Navigate options");
            telemetry.addData("", "A or RIGHT: Confirm selection");
            if (currentStep > 0) {
                telemetry.addData("", "B or LEFT: Go back");
            }
        } else {
            // All selections complete
            telemetry.addData("--- READY TO START ---", "");
            telemetry.addData("Final Configuration:", 
                             String.format("%s %s %s", 
                                         selectedAlliance == CommonDefs.Alliance.BLUE ? "BLUE" : "RED",
                                         selectedPosition == CommonDefs.PositionType.CLOSE ? "CLOSE" : "FAR",
                                         getRowsDisplayText(selectedRows)));
            telemetry.addData("", "");
            telemetry.addData("Controls:", "START: Begin autonomous");
            telemetry.addData("", "B or LEFT: Go back to modify");
        }
        
        telemetry.update();
    }
    
    private String getRowsDisplayText(CommonDefs.AutonRowsToCollect rows) {
        if (rows == CommonDefs.AutonRowsToCollect.ROS_1) {
            return "1 ROW";
        } else if (rows == CommonDefs.AutonRowsToCollect.ROS_2) {
            return "2 ROWS";
        } else {
            return "3 ROWS";
        }
    }
    
    private void savePoseToSharedPreferences() {
        // Get final pose from static holder
        Pose finalPose = RobotState.getLastAutonPose();
        if (finalPose != null) {
            SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences("robot_state", 0);
            SharedPreferences.Editor editor = prefs.edit();
            editor.putFloat("pose_x", (float) finalPose.getX());
            editor.putFloat("pose_y", (float) finalPose.getY());
            editor.putFloat("pose_heading", (float) finalPose.getHeading());
            editor.putLong("pose_timestamp", System.currentTimeMillis());
            
            // Save alliance information
            editor.putString("alliance", selectedAlliance == CommonDefs.Alliance.BLUE ? "BLUE" : "RED");
            editor.putString("position", selectedPosition == CommonDefs.PositionType.CLOSE ? "CLOSE" : "FAR");
            editor.putString("rows", selectedRows == CommonDefs.AutonRowsToCollect.ROS_1 ? "1" : 
                            selectedRows == CommonDefs.AutonRowsToCollect.ROS_2 ? "2" : "3");
            
            editor.apply();
            
            telemetry.addData("Pose Saved", "X:%.1f Y:%.1f H:%.1f°", 
                             finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading()));
            telemetry.update();
        }
    }
}