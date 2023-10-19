#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include "main.h"

#define BUFFER_SIZE 128

bool debug = false; // Verbose logging

bool interrupted = false; // Stops the main loop

// SIGINT Handler
void handleInterrupt() {
    Log("Interrupt handled");
    interrupted = true;
}

int main(int argc, char *argv[]) {
    Log("Begin");

    signal(SIGINT, handleInterrupt); // Attach interrupt handler

    debug = strcmp(argv[1], "--debug") == 0; // Check debug flag

    if(argc == 1 || argv[argc - 1][0] == '-') { // Check if a valid port argument has been provided
        Log("No port specified!");
        return -1;
    }

    char *port = argv[1];
    Log2V("Port: ", port);

    int fd = open(port, O_RDONLY | O_NOCTTY); // Open the port
    if (fd == -1) // File error handler
    {
        perror("open_port: Unable to open port - ");
        return -1;
    }

    if(debug) Log("Port open");

    char buffer[BUFFER_SIZE]; // Create UART buffer

    while(!interrupted) {
        int n = read(fd, buffer, sizeof(buffer)); // Read UART to buffer
        if (n < 0) {
            fputs("read failed!\n", stderr);
            close(fd);
            return -1;
        }

        if(debug) {
            Log("=====");
            Log("Buffer:");
            Log(buffer);
            Log("=====");
        }

        for(int i = 0; i < BUFFER_SIZE; i++) { // Iterate chars in buffer
            Locator_Process(buffer[i]); // Process NMEA message char-by-char
        }
    }

    close(fd); // Close the port
    return 0;
}

/*
 * Locator Listener
 * Gets called and prints a message whenever a lat/long new data is available.
 */
void listener() {
    Log("******** GPS Data (GPGLL) ********");
    Log4VT("Lat: ", GetLat(), "Lon: ", GetLon());
    Log("**********************************");
}

// Locator

/*
 * Defines the size of the builder. Shouldn't be less than the maximum length of the tokens.
 * Content between commas are considered as tokens and saved to builder
 */
#define BUILDER_SIZE 24

char locatorLat[BUILDER_SIZE + 1] = "NA"; // Stores the latitude string
char locatorLon[BUILDER_SIZE + 1] = "NA"; // Stores the longitude string

/*
 * Defines the possible states of the processor, during a sentence. Indicates what part of sentence is being parsed.
 */
const uint8_t LOCATOR_STATE_NULL = -1; // No sentence in the process
const uint8_t LOCATOR_STATE_BEGIN = 0; // New sentence
const uint8_t LOCATOR_STATE_LAT = 1; // Latitude
const uint8_t LOCATOR_STATE_LATDIR = 2; // Latitude Direction (N/S)
const uint8_t LOCATOR_STATE_LON = 3; // Longitude
const uint8_t LOCATOR_STATE_LONDIR = 4; // Longitude Direction (E/W)
const uint8_t LOCATOR_STATE_UTC = 5; // UTC Time Data
const uint8_t LOCATOR_STATE_END= 6; // End of sentence

// Current state
uint8_t state;
// Builder to temporarily store and append new chars to tokens
char builder[BUILDER_SIZE + 1];
// Current index of the builder content
uint8_t cursor = 0;

// Terminate the builder with a NULL byte
void Locator_TerminateBuilder() {
    if(cursor >= BUILDER_SIZE) builder[BUILDER_SIZE] = '\0';
    else builder[cursor] = '\0';
}

// Safely append char to the builder. Operation is cancelled if the builder array is full.
void Locator_AppendSafe(char data) {
    if(cursor >= BUILDER_SIZE) return;
    builder[cursor] = data;
    cursor++;
}

// Set the cursor to the next integer
void Locator_CursorIterate() {
    cursor = (cursor + 1) % BUILDER_SIZE;
}

// Append char to the builder.
void Locator_Append(char data) {
    builder[cursor] = data;
    Locator_CursorIterate();
}

// Reset the cursor to 0.
void Locator_CursorReset() {
    cursor = 0;
}

// Efficiently check if the token in the builder matches the provided word.
bool Locator_CheckWord(const char data[], uint8_t length) {
    if(cursor != length) return false;
    for(int i = 0; i < length; i++) {
        if(builder[i] != data[i]) {
            return false;
        }
    }
    return true;
}

// Move to next state
void Locator_SkipState() {
    state++;
}

// Handle the char from a NMEA sentence.
bool Locator_Handle(char data) {
    if(state == LOCATOR_STATE_END) state = LOCATOR_STATE_NULL; // If the previous sentence has ended, set locator state to LOCATOR_STATE_NULL.
    switch(data) { // Check for commas, dollar signs (Initial character of NMEA sentence), and other characters.
        case '$': // Beginning of a new NMEA sentence.
            if(debug) Log("New NMEA sentence");
            state = LOCATOR_STATE_BEGIN; // Set the state to LOCATOR_STATE_BEGIN
            Locator_CursorReset(); // Reset the builder cursor to prepare for upcoming tokens
            return true;
        case ',': // Separates tokens. End the current token and prepare for the next one.
            if (state == LOCATOR_STATE_BEGIN && Locator_CheckWord("GPGLL", 5)) // Check for GPGLL protocol header
                Locator_SkipState();
            else if (state == LOCATOR_STATE_LAT) { // If the current token is latitude...
                if (cursor == 0) return false; // Skip if the token is empty
                Locator_TerminateBuilder(); // Terminate the current builder
                for (int i = 0; i < 25; i++) {
                    locatorLat[i] = '\0'; // Fill locatorLat with NULL
                }
                for (int i = 0; i < cursor; i++) {
                    locatorLat[i] = builder[i]; // Save the latitude data from builder to locatorLat
                }
                Locator_SkipState();
            } else if (state == LOCATOR_STATE_LATDIR) { // Latitude direction doesn't require any post-processing
                Locator_SkipState();
            } else if (state == LOCATOR_STATE_LON) { // If the current token is longitude...
                if (cursor == 0) return false; // Skip if the token is empty
                Locator_TerminateBuilder(); // Terminate the current builder
                for (int i = 0; i < 25; i++) {
                    locatorLon[i] = '\0'; // Fill locatorLon with NULL
                }
                for (int i = 0; i < cursor; i++) {
                    locatorLon[i] = builder[i]; // Save the longitude data from builder to locatorLon
                }
                Locator_SkipState();
            } else if (state == LOCATOR_STATE_LONDIR) { // Longitude direction doesn't require any post-processing
                Locator_SkipState();
            } else if (state == LOCATOR_STATE_UTC) { // UTC data is ignored
                Locator_SkipState();
                listener();
            }
            Locator_CursorReset(); // Reset the builder cursor after each token
            break;
        default:
            if(state == LOCATOR_STATE_BEGIN) {
                Locator_Append(data); // Append sentence protocol header to the builder
            } else if (state == LOCATOR_STATE_LAT || state == LOCATOR_STATE_LON) {
                Locator_AppendSafe(data); // Safe append longitude and latitude data to the builder.
                // May cause loss of precision if the BUILDER_SIZE is not large enough.
            } else if (state == LOCATOR_STATE_LATDIR && data == 'S') { // Check if the latitude is on the South.
                for (int i = BUILDER_SIZE - 1; i >= 1; i--) {
                    locatorLat[i] = locatorLat[i - 1]; // Shift latitude by 1 character.
                }
                locatorLat[0] = '-'; // Add minus to the beginning of the latitude
            } else if (state == LOCATOR_STATE_LONDIR && data == 'W') {
                for (int i = BUILDER_SIZE - 1; i >= 1; i--) { // Check if the longitude is on the West.
                    locatorLon[i] = locatorLon[i - 1]; // Shift longitude by 1 character.
                }
                locatorLon[0] = '-'; // Add minus to the beginning of the longitude
            } else if (state == LOCATOR_STATE_UTC) {
                // TODO: UTC data isn't necessary at the moment
            }
            break;
    }
    return false;
}

// Process the NMEA sentence char
void Locator_Process(char data) {
    Locator_Handle(data);
}

// Get the latitude. Return NA if no data is available.
char* GetLat() {
    return locatorLat;
}

// Get the longitude. Return NA if no data is available.
char* GetLon() {
    return locatorLon;
}

// Log a message.
void Log(const char *msg) {
    printf("[LOCATOR] %s\n", msg);
}

// Concat and log 2 char arrays.
void Log2V(const char *c1, char *c2) {
    printf("[LOCATOR] %s%s\n", c1, c2);
}

// Concat and log 4 char arrays with a TAB char at the center.
void Log4VT(const char *c1, char *c2, char *c3, char *c4) {
    printf("[LOCATOR] %s%s\t%s%s\n", c1, c2, c3, c4);
}
