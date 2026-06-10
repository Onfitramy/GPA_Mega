#include "stm32h7xx_hal.h"

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))
#define TEST_LIST_COUNT ARRAY_LEN(test_list)

#define TEST_OUTPUT_BUFFER_SIZE 400

typedef enum {
    TEST_CATEGORY_GENERAL, // General test. These shouldnt be hardware specific and be able to run just on the CPU without any peripherals.
    TEST_CATEGORY_TESTS, // Tests for the test framework itself, including testing the CLI command execution and output formatting.
    TEST_CATEGORY_STATE_MACHINE, // Tests for the state machine, including state transitions, event handling, and state entry/exit actions.
    TEST_CATEGORY_SENSORS, // Tests for sensor readings, calibration, and data processing for all sensors (IMU, GPS, Baro, ADC).
    TEST_CATEGORY_CONTROL, // Tests for control and navigation algorithms, including EKF prediction and correction steps, and any other control logic implemented on the flight computer.
    TEST_CATEGORY_COMMUNICATION, // Tests for all communication interfaces, including InterBoardCom, USB communication, and any other communication protocols used inside the system(Not radio).
    TEST_CATEGORY_RADIO, // Tests for the radio communication between the two boards, including sending and receiving data packets, handling communication errors, and ensuring reliable data transfer. (Needs both boards to be connected and powered on)
    TEST_CATEGORY_SD, // Tests for the SD card storage system, including writing and reading data (needs SD card to be inserted and mounted)
    TEST_CATEGORY_FLASH, // Tests for the FLASH memory storage system, including writing and reading data
    TEST_CATEGORY_SPARK, // Tests for the SPARK sensor, including reading data and handling communication (needs spark to be connected)
    TEST_CATEGORY_POWER, // Tests for the power system, including monitoring voltage and current, and handling power-related events (needs power board to be connected)
    TEST_CATEGORY_CAMERA, // Tests for the camera system, including controlling camera power and recording, and handling camera-related events (needs camera to be connected)
} test_category_t;

typedef struct {
    const char *name;
    test_category_t category;
} test_category_lookup_t;

static const test_category_lookup_t test_category_lookup[] = {
    { "general",       TEST_CATEGORY_GENERAL },
    { "tests",         TEST_CATEGORY_TESTS },
    { "state_machine", TEST_CATEGORY_STATE_MACHINE },
    { "sensors",       TEST_CATEGORY_SENSORS },
    { "control",       TEST_CATEGORY_CONTROL },
    { "communication", TEST_CATEGORY_COMMUNICATION },
    { "radio",         TEST_CATEGORY_RADIO },
    { "sd",            TEST_CATEGORY_SD },
    { "flash",         TEST_CATEGORY_FLASH },
    { "spark",         TEST_CATEGORY_SPARK },
    { "power",         TEST_CATEGORY_POWER },
    { "camera",        TEST_CATEGORY_CAMERA },
};

typedef enum {
    TEST_STATUS_PASS    = 0,
    TEST_STATUS_FAIL    = 1,
    TEST_STATUS_NOT_RUN  = 2,
    TEST_STATUS_INVALID_ARG = 253,
    TEST_STATUS_NOT_FOUND = 255,
} test_status_t;

typedef struct
{
    const char * test_name;
    test_status_t test_status;
    uint32_t execution_time_ms;
} test_result_t;

typedef struct
{
    const char * suite_name;
    test_status_t overall_status;
    test_result_t * individual_results;
    size_t num_passed;
    size_t num_failed;
    size_t num_tests;
} test_suite_result_t;

typedef test_result_t (* test_function_callback_t)(char * test_write_buffer, size_t test_write_buffer_len);

typedef struct 
{
    const char * test_name;
    const char * test_help_string;
    test_function_callback_t test_function;
    test_category_t test_category;
} test_definition_t;

test_result_t single_test_run(const char * test_name);