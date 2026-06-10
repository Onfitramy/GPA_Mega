/**
  ******************************************************************************
  * File Description : 
  * This file implements a test suite for the Flight computer. The Test are called from the CLI.
  * A Test is a collection of test cases in a single function that are related to each other, for example testing the same module or subsystem. It can either pass or fail.
  * A Test suite is a collection of related tests that are executed together. Its either defined by a list of tests to run, or by a category of tests to run. 
  * The suite can either pass or fail based on the results of the individual tests, and can also provide a summary of the results.
  ******************************************************************************
  */

#include "test_app.h"

#include "string.h"
#include "stdio.h"
#include "stdbool.h"

char testOutputBuffer[TEST_OUTPUT_BUFFER_SIZE];

test_result_t test_cli(char *pcWriteBuffer, size_t xWriteBufferLen)
{
     test_result_t test_result = {.test_name = "test_cli", .test_status = TEST_STATUS_NOT_RUN, .execution_time_ms = 0};

     // Example test, replace with actual tests
     snprintf(testOutputBuffer, TEST_OUTPUT_BUFFER_SIZE, "Running CLI tests...\r\n");
     // Here you would call your actual test functions and append results to testOutputBuffer

     // For demonstration, we'll just print a success message
     snprintf(testOutputBuffer + strlen(testOutputBuffer), TEST_OUTPUT_BUFFER_SIZE - strlen(testOutputBuffer), "CLI tests completed successfully.\r\n");

     test_result.test_status = TEST_STATUS_PASS;
     test_result.execution_time_ms = 0; // Update with actual execution time
     return test_result;
}

test_result_t test_tests(char *pcWriteBuffer, size_t xWriteBufferLen)
{
     test_result_t test_result = {.test_name = "test_tests", .test_status = TEST_STATUS_NOT_RUN, .execution_time_ms = 0};

     // Example test, replace with actual tests
     snprintf(testOutputBuffer, TEST_OUTPUT_BUFFER_SIZE, "Running general tests...\r\n");
     // Here you would call your actual test functions and append results to testOutputBuffer

     // For demonstration, we'll just print a success message
     snprintf(testOutputBuffer + strlen(testOutputBuffer), TEST_OUTPUT_BUFFER_SIZE - strlen(testOutputBuffer), "General tests completed successfully.\r\n");

     test_result.test_status = TEST_STATUS_PASS;
     test_result.execution_time_ms = 0; // Update with actual execution time
     return test_result;
}

static const test_definition_t test_list[] = 
{
    {
        .test_name = "test_cli",
        .test_help_string = "Run CLI test",
        .test_function = test_cli,
        .test_category = TEST_CATEGORY_TESTS,
    },
    {
        .test_name = "test_tests",
        .test_help_string = "Run test for the test framework itself",
        .test_function = test_tests,
        .test_category = TEST_CATEGORY_TESTS,
    }
};

// This functions runs a single test
test_result_t single_test_run(const char * test_name) 
{
    if (test_name == NULL) {
        return (test_result_t){.test_status = TEST_STATUS_INVALID_ARG};
    }

    // Search for the test in the test list
    for (size_t i = 0; i < TEST_LIST_COUNT; i++) {

        if (strcmp(test_name, test_list[i].test_name) == 0) {

            // Run the test function and return the result
            test_result_t test_result = test_list[i].test_function(testOutputBuffer, TEST_OUTPUT_BUFFER_SIZE);

            printf("%s", testOutputBuffer);
            // flush stdout
            fflush(stdout);

            return test_result;
        }
    }

    return (test_result_t){.test_status = TEST_STATUS_NOT_FOUND}; // Test not found
}

bool test_category_from_string(const char *name, test_category_t *category)
{
    if ((name == NULL) || (category == NULL)) {
        return false;
    }

    for (size_t i = 0; i < ARRAY_LEN(test_category_lookup); i++) {
        if (strcmp(name, test_category_lookup[i].name) == 0) {
            *category = test_category_lookup[i].category;
            return true;
        }
    }

    return false;
}

test_suite_result_t test_suite_run(const char * suite_name) 
{
   if (suite_name == NULL) {
        return (test_suite_result_t){.overall_status = TEST_STATUS_INVALID_ARG};
    }

    test_suite_result_t suite_result = {
        .suite_name = suite_name,
        .overall_status = TEST_STATUS_NOT_RUN,
        .individual_results = NULL,
        .num_passed = 0,
        .num_failed = 0,
        .num_tests = 0
    };

    // Here you would implement logic to run a suite of tests based on the suite_name, for example by category or by a predefined list of tests.

    return suite_result;
}