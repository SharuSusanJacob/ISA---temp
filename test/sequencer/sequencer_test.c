#include <stdio.h>
#include <unistd.h> // For usleep()
#include <stdbool.h>
#include <math.h>   // For fabs()
#include <stdlib.h> // For malloc, free
#include <string.h> // For strtok, strcmp

#include "sequencer.h" // Your sequencer's header file

// --- Simulation Parameters ---
#define SIMULATION_STEP_MS 10           // Minor cycle is 10 ms (100 Hz)
#define SIMULATION_DURATION_S 60        // Run the simulation for 60 seconds
#define MAX_CSV_ROWS 6000               // Maximum number of rows in CSV (adjust as needed)
#define CSV_FILE_PATH "rollraterps.csv" // CSV file should be in the same directory as executable

// Structure to hold CSV data
typedef struct
{
    double missionTime;
    double rollRPS;
} RollRateData_t;

// Function to load CSV data
int loadRollRateCSV(const char *filename, RollRateData_t *data, int *count)
{
    FILE *file = fopen(filename, "r");
    if (file == NULL)
    {
        printf("Error: Cannot open CSV file '%s'\n", filename);
        return -1;
    }

    char line[256];
    int rowCount = 0;
    bool isFirstLine = true;

    while (fgets(line, sizeof(line), file) != NULL && rowCount < MAX_CSV_ROWS)
    {
        // Skip header line
        if (isFirstLine)
        {
            isFirstLine = false;
            continue;
        }

        // Parse CSV line: missiontime, rollRPS
        // Remove newline characters first
        char *newline = strchr(line, '\n');
        if (newline)
            *newline = '\0';
        newline = strchr(line, '\r');
        if (newline)
            *newline = '\0';

        char *token = strtok(line, ",");
        if (token == NULL)
            continue;

        // Read mission time (atof automatically handles whitespace)
        data[rowCount].missionTime = atof(token);

        // Read roll rate
        token = strtok(NULL, ",");
        if (token == NULL)
            continue;
        data[rowCount].rollRPS = atof(token);

        rowCount++;
    }

    fclose(file);
    *count = rowCount;
    return 0;
}

// Function to get roll rate at a specific mission time (linear interpolation)
double getRollRateAtTime(const RollRateData_t *data, int dataCount, double missionTime)
{
    // Find the closest data points
    if (dataCount == 0)
        return 0.0f;
    if (missionTime <= data[0].missionTime)
        return (double)data[0].rollRPS;
    if (missionTime >= data[dataCount - 1].missionTime)
        return (double)data[dataCount - 1].rollRPS;

    // Linear interpolation
    for (int i = 0; i < dataCount - 1; i++)
    {
        if (missionTime >= data[i].missionTime && missionTime <= data[i + 1].missionTime)
        {
            double t = (missionTime - data[i].missionTime) / (data[i + 1].missionTime - data[i].missionTime);
            return (double)(data[i].rollRPS + t * (data[i + 1].rollRPS - data[i].rollRPS));
        }
    }

    return (double)data[dataCount - 1].rollRPS;
}

int main(void)
{
    printf("--- Sequencer Simulation Started ---\n");

    // Load roll rate data from CSV
    RollRateData_t rollRateData[MAX_CSV_ROWS];
    int csvRowCount = 0;

    printf("Loading roll rate data from CSV...\n");
    if (loadRollRateCSV(CSV_FILE_PATH, rollRateData, &csvRowCount) != 0)
    {
        printf("Error loading CSV file. Exiting.\n");
        return 1;
    }
    printf("Loaded %d rows from CSV file.\n", csvRowCount);
    if (csvRowCount > 0)
    {
        printf("First entry: time=%.2fs, rollRate=%.6f rps\n", rollRateData[0].missionTime, rollRateData[0].rollRPS);
        if (csvRowCount > 100)
        {
            printf("Entry at 5s: time=%.2fs, rollRate=%.6f rps\n", rollRateData[500].missionTime, rollRateData[500].rollRPS);
        }
    }
    printf("\n");

    // 1. Storage for Sequencer State and Output
    SequencerState_t state;
    SequencerOutput_t output;

    // 2. Initialize the sequencer
    sequencerInit(&state);

    // --- Simulation Inputs ---
    double rollRateFp = 0.0; // Will be loaded from CSV
    // Initialize roll rate from first CSV entry (before launch)
    if (csvRowCount > 0)
    {
        rollRateFp = (double)rollRateData[0].rollRPS;
    }
    double tGo = 1000000.0; // Guidance tGo signal (starts invalid)

    // Variables for tGo simulation
    double guidStartTime = 0.0;  // Mission time when guidance starts
    bool guidanceActive = false; // Track if guidance has started

    // --- Output CSV Setup ---
    FILE *outFile = fopen("sequencer_sim_output.csv", "w");
    if (outFile == NULL)
    {
        printf("Error: Cannot create output CSV file.\n");
        return 1;
    }
    // Write Title and Header
    fprintf(outFile, "Sequencer Closed Loop Test Module\n");
    fprintf(outFile, "Mission_Time,Roll_Rate_RPS,Tgo_Seconds,State,Event_Description\n");

    // --- Interactive Launch ---
    printf("Projectile is in PRE-LAUNCH state. Waiting for OBC Reset.\n");
    printf("Fire the Projectile? (y/n): ");
    while (getchar() != 'y')
    {
        // Wait for user to press 'y'
    }
    printf("\nLaunch Command Received! T=0\n\n");
    sequencerSetOBCReset(&state, true); // Trigger the launch

    // --- Main Simulation Loop ---
    int total_cycles = SIMULATION_DURATION_S * (1000 / SIMULATION_STEP_MS);
    for (int i = 0; i < total_cycles; i++)
    {

        // --- Simulate Real-World Parameter Changes ---

        // Calculate mission time BEFORE sequencerExecute increments the cycle counter
        // Use the current cycle count (will be incremented inside sequencerExecute)
        double currentMissionTime = (double)state.mainClockCycles * 0.01; // Convert cycles to seconds

        // 1. Get roll rate from CSV data based on current mission time
        // Mission time in seconds = cycle count * 0.01 (since each cycle is 10ms)
        if (state.isT0Set)
        {
            rollRateFp = getRollRateAtTime(rollRateData, csvRowCount, currentMissionTime);
        }
        else
        {
            // Before launch, use first CSV entry
            if (csvRowCount > 0)
            {
                rollRateFp = (double)rollRateData[0].rollRPS;
            }
        }

        // 2. Simulate realistic tGo (time-to-go) from guidance
        // Based on reference model analysis:
        // - Before guidance start: tGo = 1000000 (invalid)
        // - After guidance start: tGo starts at ~25s and decreases
        // - Proximity triggers when tGo < 3.5s (happens ~27.5s after guidance start)

        // Check if guidance start flag has been sent
        if (state.isGuidStartFlagSent && !guidanceActive)
        {
            // Guidance just started - record the time and activate
            guidStartTime = currentMissionTime;
            guidanceActive = true;
            tGo = 25.0f; // Initial tGo value when guidance starts
        }

        if (guidanceActive)
        {
            // Simulate tGo decreasing over time
            // Time elapsed since guidance started
            double timeSinceGuidStart = currentMissionTime - guidStartTime;

            // Simulate tGo countdown (approximately linear for simplicity)
            // Reference shows ~27.5s from guidance start to tGo=3.5s
            // So tGo decreases from 25s to 3.5s over 27.5s
            // Rate: (25 - 3.5) / 27.5 ≈ 0.78 seconds per second
            tGo = 25.0 - (double)(timeSinceGuidStart * 0.78);

            // Ensure tGo doesn't go negative
            if (tGo < 0.0f)
            {
                tGo = 0.0f;
            }
        }
        else
        {
            // Before guidance starts, tGo is invalid
            tGo = 1000000.0f;
        }

        // --- Execute the sequencer for one cycle ---
        sequencerExecute(&state, rollRateFp, tGo, &output);

        // --- Determine State and Event Description for CSV ---
        int currentState = 0;
        char fullEventDesc[512]; // Buffer to hold state + flags
        const char *stateBase = "T0_STATE_ACTIVE";

        if (state.isT3Set)
        {
            currentState = 3;
            stateBase = "T3_STATE_ACTIVE";
        }
        else if (state.isT2Set)
        {
            currentState = 2;
            stateBase = "T2_STATE_ACTIVE";
        }
        else if (state.isT1Set)
        {
            currentState = 1;
            stateBase = "T1_STATE_ACTIVE";
        }
        else if (state.isT0Set)
        {
            currentState = 0;
            stateBase = "T0_STATE_ACTIVE";
        }

        // Start with the state base
        strcpy(fullEventDesc, stateBase);

        // Append flags if they are active
        if (output.fsaActivateFlag)
        {
            strcat(fullEventDesc, " | FSA_Flag");
        }
        if (output.canardDeployFlag)
        {
            strcat(fullEventDesc, " | Canard_Deploy");
        }
        if (output.canardControlFlag)
        {
            strcat(fullEventDesc, " | Canard_Control");
        }
        if (output.sendGuidStartFlag)
        {
            strcat(fullEventDesc, " | Guid_Start");
        }
        if (output.enableProximitySensor)
        {
            strcat(fullEventDesc, " | Prox_Sensor");
        }

        // --- Write to CSV ---
        fprintf(outFile, "%.2f,%.6f,%.6f,%d,%s\n",
                currentMissionTime,
                rollRateFp,
                tGo,
                currentState,
                fullEventDesc);

        // --- Print the Status for Observation ---
        printf("Cycle: %-5u | Mission Time: %6.2fs | Roll Rate: %.6f rps | tGo: %8.2fs | ",
               state.mainClockCycles,
               currentMissionTime,
               (double)rollRateFp,
               (double)tGo);

        // Print the current active phase
        if (state.isT3Set)
            printf("Phase: T3 | ");
        else if (state.isT2Set)
            printf("Phase: T2 | ");
        else if (state.isT1Set)
            printf("Phase: T1 | ");
        else if (state.isT0Set)
            printf("Phase: T0 | ");

        // Print any commands (outputs) that were generated this cycle
        printf("Commands: [ ");
        if (output.fsaActivateFlag)
            printf("FSA Activate Flag ");
        if (output.canardDeployFlag)
            printf("Canard Deploy Flag ");
        if (output.canardControlFlag)
            printf("Canard Control Flag ");
        if (output.sendGuidStartFlag)
            printf("Guidance start Flag ");
        if (output.enableProximitySensor)
            printf("Proximity Enable Flag ");
        printf("]\n");

        // Stop the simulation if the final phase is complete
        if (state.isT3Set && output.enableProximitySensor)
        {
            printf("\n--- Mission Complete: Proximity Sensor Enabled ---\n");
            break;
        }

        // Simulate the 10 ms delay between cycles
        usleep(SIMULATION_STEP_MS * 1000);
    }

    fclose(outFile);
    printf("\nResults saved to 'sequencer_sim_output.csv'\n");
    printf("\n--- Sequencer Simulation Finished ---\n");
    return 0;
}