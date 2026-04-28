#include "agnocast_cie_thread_configurator/non_ros_thread_info_ipc.hpp"

#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <cstddef>
#include <string>

using agnocast_cie_thread_configurator::fill_abstract_sockaddr;
using agnocast_cie_thread_configurator::kNonRosThreadNameMax;
using agnocast_cie_thread_configurator::ThreadNameValidation;
using agnocast_cie_thread_configurator::validate_thread_name;

TEST(ValidateThreadName, TypicalNamesAreOk)
{
  EXPECT_EQ(validate_thread_name("worker"), ThreadNameValidation::kOk);
  EXPECT_EQ(validate_thread_name("non_ros_thread_1"), ThreadNameValidation::kOk);
}

TEST(ValidateThreadName, EmptyIsOk)
{
  EXPECT_EQ(validate_thread_name(""), ThreadNameValidation::kOk);
}

TEST(ValidateThreadName, ExactlyMaxLengthIsOk)
{
  std::string at_max(kNonRosThreadNameMax, 'a');
  EXPECT_EQ(validate_thread_name(at_max), ThreadNameValidation::kOk);
}

TEST(ValidateThreadName, OneOverMaxIsTooLong)
{
  std::string over_max(kNonRosThreadNameMax + 1, 'a');
  EXPECT_EQ(validate_thread_name(over_max), ThreadNameValidation::kTooLong);
}

TEST(ValidateThreadName, FarOverMaxIsTooLong)
{
  std::string far_over(1024, 'a');
  EXPECT_EQ(validate_thread_name(far_over), ThreadNameValidation::kTooLong);
}

TEST(ValidateThreadName, EmbeddedNulAtStart)
{
  std::string with_nul("\0abc", 4);
  EXPECT_EQ(validate_thread_name(with_nul), ThreadNameValidation::kEmbeddedNul);
}

TEST(ValidateThreadName, EmbeddedNulInMiddle)
{
  std::string with_nul("ab\0cd", 5);
  EXPECT_EQ(validate_thread_name(with_nul), ThreadNameValidation::kEmbeddedNul);
}

TEST(ValidateThreadName, EmbeddedNulAtEnd)
{
  std::string with_nul("abc\0", 4);
  EXPECT_EQ(validate_thread_name(with_nul), ThreadNameValidation::kEmbeddedNul);
}

TEST(FillAbstractSockaddr, FormatIsCorrect)
{
  sockaddr_un addr;
  const socklen_t len = fill_abstract_sockaddr(addr, "test_socket");

  EXPECT_EQ(addr.sun_family, AF_UNIX);
  EXPECT_EQ(addr.sun_path[0], '\0');
  EXPECT_EQ(std::string(addr.sun_path + 1, 11), "test_socket");
  EXPECT_EQ(len, static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + 11));
}

TEST(FillAbstractSockaddr, EmptyName)
{
  sockaddr_un addr;
  const socklen_t len = fill_abstract_sockaddr(addr, "");

  EXPECT_EQ(addr.sun_family, AF_UNIX);
  EXPECT_EQ(addr.sun_path[0], '\0');
  EXPECT_EQ(len, static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1));
}
