#!/bin/bash
#
# lcm_batch_processor.sh - Process all LCM files in a directory
#
# Usage: ./lcm_batch_processor.sh [options] <directory>
#
# Options:
#   -c      Generate C code
#   -cpp    Generate C++ code
#   -j      Generate Java code
#   -p      Generate Python code
#   -l      Generate Lua code
#   -cs     Generate C# code
#   -o DIR  Output directory for generated files
#   -h      Show this help message
#
# Example: ./lcm_batch_processor.sh -cpp -j ~/lcm_files/

# Default values
GENERATE_C=false
GENERATE_CPP=false
GENERATE_JAVA=false
GENERATE_PYTHON=false
GENERATE_LUA=false
GENERATE_CSHARP=false
OUTPUT_DIR=""
VERBOSE=false

# Function to display usage
show_help() {
    echo "Usage: $0 [options] <directory>"
    echo "Process all LCM files in a directory using lcm-gen"
    echo ""
    echo "Options:"
    echo "  -c      Generate C code"
    echo "  -cpp    Generate C++ code"
    echo "  -j      Generate Java code"
    echo "  -p      Generate Python code"
    echo "  -l      Generate Lua code"
    echo "  -cs     Generate C# code"
    echo "  -o DIR  Output directory for generated files"
    echo "  -v      Verbose output"
    echo "  -h      Show this help message"
    echo ""
    echo "Example: $0 -cpp -j ~/lcm_files/"
    exit 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -c)
            GENERATE_C=true
            shift
            ;;
        -cpp)
            GENERATE_CPP=true
            shift
            ;;
        -j)
            GENERATE_JAVA=true
            shift
            ;;
        -p)
            GENERATE_PYTHON=true
            shift
            ;;
        -l)
            GENERATE_LUA=true
            shift
            ;;
        -cs)
            GENERATE_CSHARP=true
            shift
            ;;
        -o)
            shift
            if [[ $# -gt 0 ]]; then
                OUTPUT_DIR="$1"
                shift
            else
                echo "Error: Output directory not specified"
                exit 1
            fi
            ;;
        -v)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            show_help
            ;;
        *)
            DIRECTORY="$1"
            shift
            ;;
    esac
done

# Check if lcm-gen is installed
if ! command -v lcm-gen &> /dev/null; then
    echo "Error: lcm-gen command not found"
    echo "Please install LCM first: https://lcm-proj.github.io/"
    exit 1
fi

# Check if a directory was specified
if [ -z "$DIRECTORY" ]; then
    echo "Error: No directory specified"
    show_help
fi

# Check if the directory exists
if [ ! -d "$DIRECTORY" ]; then
    echo "Error: Directory '$DIRECTORY' does not exist"
    exit 1
fi

# Check if at least one language option was selected
if ! $GENERATE_C && ! $GENERATE_CPP && ! $GENERATE_JAVA && ! $GENERATE_PYTHON && ! $GENERATE_LUA && ! $GENERATE_CSHARP; then
    echo "Error: No language specified, at least one of -c, -cpp, -j, -p, -l, or -cs must be provided"
    exit 1
fi

# Create output directory if specified and doesn't exist
if [ -n "$OUTPUT_DIR" ] && [ ! -d "$OUTPUT_DIR" ]; then
    mkdir -p "$OUTPUT_DIR"
    if [ $? -ne 0 ]; then
        echo "Error: Failed to create output directory '$OUTPUT_DIR'"
        exit 1
    fi
    echo "Created output directory: $OUTPUT_DIR"
fi

# Build lcm-gen command base
LCM_GEN_CMD="lcm-gen --lazy"

# Set output directory for each language if specified
if [ -n "$OUTPUT_DIR" ]; then
    if $GENERATE_C; then
        C_OPTIONS="--c-cpath $OUTPUT_DIR --c-hpath $OUTPUT_DIR"
    fi
    if $GENERATE_CPP; then
        CPP_OPTIONS="--cpp-cpath $OUTPUT_DIR --cpp-hpath $OUTPUT_DIR"
    fi
    if $GENERATE_JAVA; then
        JAVA_OPTIONS="--jpath $OUTPUT_DIR"
    fi
    if $GENERATE_PYTHON; then
        PYTHON_OPTIONS="--ppath $OUTPUT_DIR"
    fi
    if $GENERATE_LUA; then
        LUA_OPTIONS="--lpath $OUTPUT_DIR"
    fi
    if $GENERATE_CSHARP; then
        CSHARP_OPTIONS="--csharp-path $OUTPUT_DIR"
    fi
fi

# Function to process a single LCM file
process_lcm_file() {
    local lcm_file="$1"
    local cmd="$LCM_GEN_CMD"
    
    if $GENERATE_C; then
        cmd="$cmd -c $C_OPTIONS"
    fi
    if $GENERATE_CPP; then
        cmd="$cmd --cpp $CPP_OPTIONS"
    fi
    if $GENERATE_JAVA; then
        cmd="$cmd -j $JAVA_OPTIONS"
    fi
    if $GENERATE_PYTHON; then
        cmd="$cmd -p $PYTHON_OPTIONS"
    fi
    if $GENERATE_LUA; then
        cmd="$cmd -l $LUA_OPTIONS"
    fi
    if $GENERATE_CSHARP; then
        cmd="$cmd --csharp $CSHARP_OPTIONS"
    fi
    
    cmd="$cmd $lcm_file"
    
    if $VERBOSE; then
        echo "Executing: $cmd"
    fi
    
    eval $cmd
    local status=$?
    
    if [ $status -eq 0 ]; then
        echo "Successfully processed: $lcm_file"
    else
        echo "Error processing: $lcm_file (exit code: $status)"
    fi
    
    return $status
}

# Find all .lcm files and process them
echo "Searching for LCM files in '$DIRECTORY'..."
lcm_files=$(find "$DIRECTORY" -name "*.lcm" -type f)
lcm_count=$(echo "$lcm_files" | wc -l)

if [ -z "$lcm_files" ]; then
    echo "No LCM files found in '$DIRECTORY'"
    exit 1
fi

echo "Found $lcm_count LCM files to process"

# Process each LCM file
success_count=0
error_count=0

for lcm_file in $lcm_files; do
    process_lcm_file "$lcm_file"
    if [ $? -eq 0 ]; then
        ((success_count++))
    else
        ((error_count++))
    fi
done

# Print summary
echo ""
echo "Processing complete!"
echo "  Total files: $lcm_count"
echo "  Successfully processed: $success_count"
echo "  Errors: $error_count"

if [ $error_count -gt 0 ]; then
    exit 1
fi

exit 0
