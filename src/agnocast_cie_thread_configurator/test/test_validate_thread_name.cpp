#include "agnocast_cie_thread_configurator/cie_thread_configurator.hpp"

#include <gtest/gtest.h>

#include <string>

using agnocast_cie_thread_configurator::kNonRosThreadNameMax;
using agnocast_cie_thread_configurator::ThreadNameValidation;
using agnocast_cie_thread_configurator::validate_thread_name;

TEST(ValidateThreadName, EmptyStringIsOk)
{
  EXPECT_EQ(validate_thread_name(""), ThreadNameValidation::kOk);
}

TEST(ValidateThreadName, TypicalNameIsOk)
{
  EXPECT_EQ(validate_thread_name("worker_thread_1"), ThreadNameValidation::kOk);
}

TEST(ValidateThreadName, ExactlyMaxLengthIsOk)
{
  EXPECT_EQ(
    validate_thread_name(std::string(kNonRosThreadNameMax, 'a')), ThreadNameValidation::kOk);
}

TEST(ValidateThreadName, OneOverMaxLengthIsTooLong)
{
  EXPECT_EQ(
    validate_thread_name(std::string(kNonRosThreadNameMax + 1, 'a')),
    ThreadNameValidation::kTooLong);
}

TEST(ValidateThreadName, EmbeddedNulIsRejected)
{
  std::string name = "abc";
  name.push_back('\0');
  name += "def";
  EXPECT_EQ(validate_thread_name(name), ThreadNameValidation::kEmbeddedNul);
}

TEST(ValidateThreadName, LeadingNulIsRejected)
{
  std::string name;
  name.push_back('\0');
  name += "trailing";
  EXPECT_EQ(validate_thread_name(name), ThreadNameValidation::kEmbeddedNul);
}

TEST(ValidateThreadName, TooLongTakesPrecedenceOverEmbeddedNul)
{
  std::string name(kNonRosThreadNameMax + 1, 'a');
  name[10] = '\0';
  EXPECT_EQ(validate_thread_name(name), ThreadNameValidation::kTooLong);
}

TEST(ValidateThreadName, MaxLengthWithTrailingNulIsRejected)
{
  // Boundary: size() == kNonRosThreadNameMax (size check passes) and the very last byte is
  // an embedded NUL, so the embedded-NUL check must still fire at the last position.
  std::string name(kNonRosThreadNameMax - 1, 'a');
  name.push_back('\0');
  ASSERT_EQ(name.size(), kNonRosThreadNameMax);
  EXPECT_EQ(validate_thread_name(name), ThreadNameValidation::kEmbeddedNul);
}

TEST(ValidateThreadName, NullptrIsRejected)
{
  EXPECT_EQ(
    validate_thread_name(static_cast<const char *>(nullptr)), ThreadNameValidation::kNullptr);
}

TEST(ValidateThreadName, CStringOverloadDelegates)
{
  EXPECT_EQ(validate_thread_name("worker"), ThreadNameValidation::kOk);
}
