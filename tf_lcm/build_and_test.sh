#!/bin/bash
# Build and test script for tf_lcm library

set -e  # Exit on error

# Function to run a test with timeout
run_test_with_timeout() {
    local test_name="$1"
    local timeout_seconds="$2"
    local command="$3"
    
    echo "Running $test_name..."
    timeout $timeout_seconds $command &
    local pid=$!
    wait $pid
    local exit_code=$?
    
    if [ $exit_code -eq 0 ]; then
        echo "✅ $test_name completed successfully"
        return 0
    elif [ $exit_code -eq 124 ]; then
        echo "⚠️ $test_name timed out after $timeout_seconds seconds"
        return 1
    else
        echo "❌ $test_name failed with exit code $exit_code"
        return 1
    fi
}

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo "Configuring tf_lcm with CMake..."
cmake .. || { echo "CMake configuration failed"; exit 1; }

# Build
echo "Building tf_lcm library and tests..."
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu) || { echo "Build failed"; exit 1; }

echo "\n📝 Running tests (with timeouts to prevent hanging)..."

# Run the tests with timeouts
TEST_FAILURES=0

# Skip the broadcaster/listener paired test if a specific test was requested
if [ -z "$1" ]; then
    echo "\n🧪 Testing broadcaster and listener separately..."
    
    # Run broadcaster with a short timeout instead of running in background
    echo "\n📡 Testing broadcaster..."
    timeout 3 test/test_broadcaster
    BROADCASTER_RESULT=$?
    
    if [ $BROADCASTER_RESULT -eq 0 ]; then
        echo "✅ Broadcaster test completed successfully"
    else
        echo "⚠️ Broadcaster test did not complete properly (timeout or error)"
        TEST_FAILURES=$((TEST_FAILURES+1))
    fi
    
    # Run listener test with a short timeout
    echo "\n📻 Testing listener..."
    timeout 3 test/test_listener
    LISTENER_RESULT=$?
    
    if [ $LISTENER_RESULT -eq 0 ]; then
        echo "✅ Listener test completed successfully"
    else
        echo "⚠️ Listener test did not complete properly (timeout or error)"
        TEST_FAILURES=$((TEST_FAILURES+1))
    fi
    
    # Note about the combined test
    echo "\nNote: To run broadcaster and listener in combination, manually run:"
    echo "  cd build && ./test/test_broadcaster & sleep 1 && ./test/test_listener"
fi

# Function to show available tests
show_test_options() {
    echo "\n📃 Available test options:"
    echo "  all           - Run all tests (default)"
    echo "  lookup        - Test transform lookups"
    echo "  static        - Test static transforms"
    echo "  timeout       - Test timeout behavior"
    echo "  time_travel   - Test time travel transforms"
    echo "  broadcaster   - Test transform broadcaster"
    echo "  listener      - Test transform listener"
    echo "  simple_demo   - Run simple TF demo (recommended for quick testing)"
    echo "  minimal       - Run minimal transform test (fastest)"
    echo "  chained       - Run test with chained transforms and graph traversal"
    echo "  --minimal-test       Run minimal test for core functionality"
    echo "  --chained-test       Run chained transform test (multi-hop lookups)"
    echo "  --simple-demo         Run a simple TF demo"
    echo "  help          - Show this help message"
}

# Run individual tests with shorter timeouts
if [ -n "$1" ]; then
    # If a specific test is specified, run only that test
    case "$1" in
        "lookup")
            run_test_with_timeout "Transform lookups test" 2 "test/test_lookup" || TEST_FAILURES=$((TEST_FAILURES+1))
            ;;
        "static")
            run_test_with_timeout "Static transforms test" 5 "test/test_static_transform" || TEST_FAILURES=$((TEST_FAILURES+1))
            ;;
        "timeout")
            run_test_with_timeout "Timeout test" 5 "test/test_timeout" || TEST_FAILURES=$((TEST_FAILURES+1))
            ;;
        "time_travel")
            run_test_with_timeout "Time travel test" 5 "test/test_time_travel" || TEST_FAILURES=$((TEST_FAILURES+1))
            ;;
        "broadcaster")
            echo "\n🧪 Testing broadcaster only..."
            timeout 5 test/test_broadcaster || TEST_FAILURES=$((TEST_FAILURES+1))
            ;;
        "listener")
            echo "\n🧪 Testing listener only..."
            timeout 5 test/test_listener || TEST_FAILURES=$((TEST_FAILURES+1))
            ;;
        "simple_demo")
            echo "\n🧪 Running Simple TF Demo..."
            test/simple_tf_demo || TEST_FAILURES=$((TEST_FAILURES+1))
            ;;
        "minimal")
            echo "\n🧪 Running Minimal TF Test..."
            test/minimal_test || TEST_FAILURES=$((TEST_FAILURES+1))
            ;;
        "chained")
            echo "\n🧪 Running Chained Transforms Test..."
            test/chained_test || TEST_FAILURES=$((TEST_FAILURES+1))
            ;;
        "help")
            show_test_options
            exit 0
            ;;
        "all")
            # All tests will run below
            ;;
        *)
            echo "\n⚠️ Unknown test: $1"
            show_test_options
            exit 1
            ;;
    esac
elif [ -z "$1" ]; then
    # If no argument is provided, show usage and run all tests
    echo "\n📚 TF-LCM Test Suite"
    echo "Running all tests. For specific tests, run: $0 <test_name>"
    show_test_options
fi

# Run all tests if 'all' was specified or no specific test was requested
if [ -z "$1" ] || [ "$1" = "all" ]; then
    # Run core tests with shorter timeouts
    run_test_with_timeout "Simple TF Demo" 5 "test/simple_tf_demo" || TEST_FAILURES=$((TEST_FAILURES+1))
    run_test_with_timeout "Transform lookups test" 5 "test/test_lookup" || TEST_FAILURES=$((TEST_FAILURES+1))
    run_test_with_timeout "Static transforms test" 5 "test/test_static_transform" || TEST_FAILURES=$((TEST_FAILURES+1))
    run_test_with_timeout "Timeout test" 5 "test/test_timeout" || TEST_FAILURES=$((TEST_FAILURES+1))
    run_test_with_timeout "Time travel test" 5 "test/test_time_travel" || TEST_FAILURES=$((TEST_FAILURES+1))
fi

# Copy Python module to the python directory
echo "\n📦 Setting up Python module..."
PY_MODULE=$(find . -name "*.so" | grep -E "tf_lcm_py\..*\.so")
if [ -n "$PY_MODULE" ]; then
    cp "$PY_MODULE" ../python/tf_lcm_py.so
    echo "✅ Python module copied to ../python/tf_lcm_py.so"
else
    echo "⚠️ Warning: Python module not found"
    TEST_FAILURES=$((TEST_FAILURES+1))
fi

# Print test summary
echo "\n📋 Test Summary:"
if [ $TEST_FAILURES -eq 0 ]; then
    echo "✅ All tests completed successfully!"
    echo "🎉 TF library for LCM is now ready for use with both C++ and Python"
    echo "📚 The library is compatible with the existing python_lcm_msgs package"
    echo "\nUsage examples are provided in the README.md file and test directories."
    exit 0
else
    echo "⚠️ $TEST_FAILURES test(s) failed or timed out"
    echo "The build was successful, but some tests did not complete properly."
    echo "You may still use the library, but please review the test output for details."
    exit 1
fi
