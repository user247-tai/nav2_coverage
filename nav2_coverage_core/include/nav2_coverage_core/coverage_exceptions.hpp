// MIT License

// Copyright (c) 2026 Nguyen Thanh Tai

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef NAV2_COVERAGE_CORE__COVERAGE_EXCEPTIONS_HPP_
#define NAV2_COVERAGE_CORE__COVERAGE_EXCEPTIONS_HPP_

#include <stdexcept>

namespace nav2_coverage_core
{

class CoverageException : public std::runtime_error
{
public:
  explicit CoverageException(const std::string & description)
  : std::runtime_error(description) {}
};

class FailedToPlanCoverage : public CoverageException
{
public:
  explicit FailedToPlanCoverage(const std::string & description)
  : CoverageException(description) {}
};

class FailedToFollowPlanCoverage : public CoverageException
{
public:
  explicit FailedToFollowPlanCoverage(const std::string & description)
  : CoverageException(description) {}
};

class FailedToRecoverCoverage : public CoverageException
{
public:
  explicit FailedToRecoverCoverage(const std::string & description)
  : CoverageException(description) {}
}; 

class TimeoutCreatePoses : public CoverageException
{
public:
  explicit TimeoutCreatePoses(const std::string & description)
  : CoverageException(description) {}
}; 

}

#endif // NAV2_COVERAGE_CORE__COVERAGE_EXCEPTIONS_HPP_