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

#include "main_app.h"

#include "string.h"
#include "stdio.h"
#include "stdbool.h"

char testOutputBuffer[TEST_OUTPUT_BUFFER_SIZE];

test_result_t test_cli(char *test_write_buffer, size_t test_write_buffer_len)
{
     test_result_t test_result = {.test_name = "test_cli", .test_status = TEST_STATUS_NOT_RUN, .execution_time_ms = 0};

     // Example test, replace with actual tests
     snprintf(test_write_buffer, test_write_buffer_len, "Running CLI tests...\r\n");
     // Here you would call your actual test functions and append results to test_write_buffer

     // For demonstration, we'll just print a success message
     snprintf(test_write_buffer + strlen(test_write_buffer), test_write_buffer_len - strlen(test_write_buffer), "CLI tests completed successfully.\r\n");

     test_result.test_status = TEST_STATUS_PASS;
     test_result.execution_time_ms = 0; // Update with actual execution time
     return test_result;
}

test_result_t test_tests(char *test_write_buffer, size_t test_write_buffer_len)
{
     test_result_t test_result = {.test_name = "test_tests", .test_status = TEST_STATUS_NOT_RUN, .execution_time_ms = 0};

     // Example test, replace with actual tests
     snprintf(test_write_buffer, test_write_buffer_len, "Running general tests...\r\n");
     // Here you would call your actual test functions and append results to test_write_buffer

     // For demonstration, we'll just print a success message
     snprintf(test_write_buffer + strlen(test_write_buffer), test_write_buffer_len - strlen(test_write_buffer), "General tests completed successfully.\r\n");

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
test_result_t test_run(const char * test_name, char *test_write_buffer, size_t test_write_buffer_len) 
{
    if (test_name == NULL) {
        return (test_result_t){.test_status = TEST_STATUS_INVALID_ARG};
    }

    // Search for the test in the test list
    for (size_t i = 0; i < TEST_LIST_COUNT; i++) {

        if (strcmp(test_name, test_list[i].test_name) == 0) {

            // Run the test function and return the result
            test_result_t test_result = test_list[i].test_function(test_write_buffer, test_write_buffer_len);

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

test_result_t single_test_run(const char * test_name) 
{
    test_result_t result = test_run(test_name, testOutputBuffer, TEST_OUTPUT_BUFFER_SIZE);
    
    printf("%s", testOutputBuffer);
    // flush stdout
    fflush(stdout);

    return result;
}

test_suite_result_t test_suite_run(const char * suite_name) 
{
   if (suite_name == NULL) {
        return (test_suite_result_t){.overall_status = TEST_STATUS_INVALID_ARG};
    }

    test_result_t testSuiteResults[MAX_TESTS_IN_SUITE];
    test_suite_result_t suite_result = {
        .suite_name = suite_name,
        .overall_status = TEST_STATUS_NOT_RUN,
        .individual_results = testSuiteResults,
        .num_passed = 0,
        .num_failed = 0,
        .num_tests = 0
    };

    // Check if suit name is in the category lookup, if so run all tests in that category
    test_category_t category;
    uint8_t suite_type; // 0 for category, 1 for predefined list, 2 for invalid 

    if (test_category_from_string(suite_name, &category)) {
        suite_type = 0;
    } else {
        suite_type = 2; // Invalid suite name
        return suite_result;
    }

    // Count how many tests are in the suite
    for (size_t i = 0; i < TEST_LIST_COUNT; i++) {
        if (test_list[i].test_category == category) {
            suite_result.num_tests++;
        }
    }

    if (suite_result.num_tests == 0) {
        suite_result.overall_status = TEST_STATUS_NOT_FOUND; // No tests found for this category
        return suite_result;
    }

    // Run the tests in the suite
    for (size_t i = 0; i < suite_result.num_tests; i++) {
        suite_result.individual_results[i] = test_run(test_list[i].test_name, testOutputBuffer, TEST_OUTPUT_BUFFER_SIZE);
        if (suite_result.individual_results[i].test_status == TEST_STATUS_PASS) {
            suite_result.num_passed++;
        } else {
            suite_result.num_failed++;
        }
        
        printf("Test %s: %s\r\n", test_list[i].test_name, (suite_result.individual_results[i].test_status == TEST_STATUS_PASS) ? "PASS" : "FAIL");
        // flush stdout
        fflush(stdout);
    }

    suite_result.overall_status = (suite_result.num_failed == 0) ? TEST_STATUS_PASS : TEST_STATUS_FAIL;

    printf("Test suite %s completed: %d/%d tests passed\r\n", suite_name, suite_result.num_passed, suite_result.num_tests);
    // flush stdout
    fflush(stdout);

    return suite_result;
}