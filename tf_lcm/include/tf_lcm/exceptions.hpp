#pragma once

#include <stdexcept>
#include <string>

namespace tf_lcm {

/**
 * @brief Base class for all tf_lcm exceptions
 */
class TransformException : public std::runtime_error {
public:
    explicit TransformException(const std::string& error_msg)
        : std::runtime_error(error_msg) {}
};

/**
 * @brief Exception thrown when a transformation lookup fails
 */
class LookupException : public TransformException {
public:
    explicit LookupException(const std::string& error_msg)
        : TransformException(error_msg) {}
};

/**
 * @brief Exception thrown when a connectivity error occurs
 */
class ConnectivityException : public TransformException {
public:
    explicit ConnectivityException(const std::string& error_msg)
        : TransformException(error_msg) {}
};

/**
 * @brief Exception thrown when the requested transform is not valid
 */
class InvalidArgumentException : public TransformException {
public:
    explicit InvalidArgumentException(const std::string& error_msg)
        : TransformException(error_msg) {}
};

/**
 * @brief Exception thrown when a time out occurs
 */
class TimeoutException : public TransformException {
public:
    explicit TimeoutException(const std::string& error_msg)
        : TransformException(error_msg) {}
};

/**
 * @brief Exception thrown when a time value is outside the cache
 */
class ExtrapolationException : public TransformException {
public:
    explicit ExtrapolationException(const std::string& error_msg)
        : TransformException(error_msg) {}
};

} // namespace tf_lcm
