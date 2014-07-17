// Last changed on 2011-04-22.


#include <string>

#include <TRTK/ErrorObj.hpp>

#include "unit_test.hpp"


class ErrorObjTestClass
{
public:
        void testConstructor1()
        {
            TRTK::ErrorObj error;
            throw error;
        }

        void testConstructor2(const std::string & error_message)
        {
            TRTK::ErrorObj error(error_message);
            throw error;
        }

        void testConstructor3(const std::string error_message,
                              const std::string class_name)
        {
            TRTK::ErrorObj error(error_message, class_name);
            throw error;
        }

        void testConstructor4(const std::string error_message,
                              const std::string class_name,
                              const std::string function_name,
                              const int error_code)
        {
            TRTK::ErrorObj error(error_message, class_name, function_name, error_code);
            throw error;
        }

        void testMethods()
        {
            TRTK::ErrorObj error;

            error.setErrorMessage("An error message.");
            error.setClassName("Class name.");
            error.setFunctionName("Function name.");
            error.setErrorCode(1);

            throw error;
        }
};


void unit_test_ErrorObj()
{
    HEADING(ErrorObj)


    SUBHEADING(Constructors)


        ErrorObjTestClass test;

        START_TEST

            try
            {
                test.testConstructor1();
            }
            catch (TRTK::ErrorObj & error)
            {
                assert(error.getErrorMessage() == "");
                assert(error.getClassName() == "");
                assert(error.getFunctionName() == "");
                assert(error.getErrorCode() == 0);
            }

        STOP_TEST


        START_TEST

            try
            {
                test.testConstructor2("An error message.");
            }
            catch (TRTK::ErrorObj & error)
            {
                assert(error.getErrorMessage() == "An error message.");
                assert(error.getClassName() == "");
                assert(error.getFunctionName() == "");
                assert(error.getErrorCode() == 0);
            }

        STOP_TEST


        START_TEST

            try
            {
                test.testConstructor3("An error message.", "Class name.");
            }
            catch (TRTK::ErrorObj & error)
            {
                assert(error.getErrorMessage() == "An error message.");
                assert(error.getClassName() == "Class name.");
                assert(error.getFunctionName() == "");
                assert(error.getErrorCode() == 0);
            }

        STOP_TEST


        START_TEST

            try
            {
                test.testConstructor4("An error message.",
                                      "Class name.",
                                      "Function name.",
                                      1);
            }
            catch (TRTK::ErrorObj & error)
            {
                assert(error.getErrorMessage() == "An error message.");
                assert(error.getClassName() == "Class name.");
                assert(error.getFunctionName() == "Function name.");
                assert(error.getErrorCode() == 1);
            }

        STOP_TEST


    SUBHEADING(Methods)


        START_TEST

            // Checks methods.

            try
            {
                test.testMethods();
            }
            catch (TRTK::ErrorObj & error)
            {
                assert(error.getErrorMessage() == "An error message.");
                assert(error.getClassName() == "Class name.");
                assert(error.getFunctionName() == "Function name.");
                assert(error.getErrorCode() == 1);
            }

        STOP_TEST


        START_TEST

            // Check virtual what().

            try
            {
                test.testMethods();
            }
            catch (std::exception & error)
            {
                std::string error_message = error.what();
                assert(error_message == "An error message.");
            }

        STOP_TEST


        START_TEST

            // Check non-verbose what().

            try
            {
                test.testMethods();
            }
            catch (TRTK::ErrorObj & error)
            {
                std::string error_message = error.what(TRTK::ErrorObj::NON_VERBOSE);
                assert(error_message == "An error message.");
            }

        STOP_TEST


        START_TEST

            // Check verbose what().

            try
            {
                test.testMethods();
            }
            catch (TRTK::ErrorObj & error)
            {
                std::string error_message = error.what(TRTK::ErrorObj::NON_VERBOSE);
                assert(error_message.length() != 0);
            }

        STOP_TEST
}
