// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Unit tests for `read_ros2_node_names()`, the reader half of the channel the discovery agent
// writes (see `ros2_node_registry.py`). The writer's own tests live in that package; here we pin
// the parsing contract the two sides share, plus the two ways the list can be missing (absent file,
// stale file), because those decide whether `NodeGraph::get_node_names()` falls back to reporting
// every Agnocast node.

#include "agnocast/internal/ros2_node_registry_reader.hpp"

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <utime.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{

constexpr uint64_t kNsInode = 4026531839;
constexpr uint32_t kDomainId = 7;

class Ros2NodeRegistryReaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char tmpl[] = "/tmp/agnocast_r2nr_test_XXXXXX";
    const char * created = mkdtemp(tmpl);
    ASSERT_NE(created, nullptr);
    base_dir_ = created;
    agnocast::internal::set_ros2_node_registry_base_dir_for_test(base_dir_);
    ns_dir_ = base_dir_ + "/" + std::to_string(kNsInode);
    std::filesystem::create_directories(ns_dir_);
    path_ = ns_dir_ + "/" + std::to_string(kDomainId);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(base_dir_, ec);
  }

  void write_file(const std::string & content) const
  {
    std::ofstream out(path_, std::ios::trunc);
    ASSERT_TRUE(out.good());
    out << content;
  }

  // Backdates the file so the reader sees it as no longer refreshed.
  void age_file(const int seconds) const
  {
    struct stat st = {};
    ASSERT_EQ(::stat(path_.c_str(), &st), 0);
    struct utimbuf times = {};
    times.actime = st.st_atime - seconds;
    times.modtime = st.st_mtime - seconds;
    ASSERT_EQ(::utime(path_.c_str(), &times), 0);
  }

  std::string base_dir_;
  std::string ns_dir_;
  std::string path_;
};

TEST_F(Ros2NodeRegistryReaderTest, ReturnsNulloptWhenNoAgentWrote)
{
  EXPECT_FALSE(agnocast::internal::read_ros2_node_names(kNsInode, kDomainId).has_value());
}

// A different domain's agent must not answer for ours.
TEST_F(Ros2NodeRegistryReaderTest, ReturnsNulloptForAnotherDomain)
{
  write_file("/\ttalker\n");

  EXPECT_FALSE(agnocast::internal::read_ros2_node_names(kNsInode, kDomainId + 1).has_value());
}

TEST_F(Ros2NodeRegistryReaderTest, ComposesFullyQualifiedNames)
{
  write_file("/\ttalker\n/sensing\tlistener\n");

  const auto names = agnocast::internal::read_ros2_node_names(kNsInode, kDomainId);

  ASSERT_TRUE(names.has_value());
  EXPECT_EQ(*names, (std::vector<std::string>{"/talker", "/sensing/listener"}));
}

// An empty namespace field means the root namespace, as rclpy reports it for a node created
// without one.
TEST_F(Ros2NodeRegistryReaderTest, TreatsEmptyNamespaceAsRoot)
{
  write_file("\ttalker\n");

  const auto names = agnocast::internal::read_ros2_node_names(kNsInode, kDomainId);

  ASSERT_TRUE(names.has_value());
  EXPECT_EQ(*names, (std::vector<std::string>{"/talker"}));
}

// Two nodes sharing a name are two nodes; collapsing them is exactly what this channel exists to
// avoid.
TEST_F(Ros2NodeRegistryReaderTest, KeepsDuplicateNames)
{
  write_file("/\ttalker\n/\ttalker\n");

  const auto names = agnocast::internal::read_ros2_node_names(kNsInode, kDomainId);

  ASSERT_TRUE(names.has_value());
  EXPECT_EQ(*names, (std::vector<std::string>{"/talker", "/talker"}));
}

TEST_F(Ros2NodeRegistryReaderTest, ReturnsEmptyListWhenNoRos2NodeIsRunning)
{
  write_file("");

  const auto names = agnocast::internal::read_ros2_node_names(kNsInode, kDomainId);

  ASSERT_TRUE(names.has_value());
  EXPECT_TRUE(names->empty());
}

// Forward compatibility: a newer writer may append fields to the line.
TEST_F(Ros2NodeRegistryReaderTest, IgnoresExtraFields)
{
  write_file("/\ttalker\t1234\tsomething\n");

  const auto names = agnocast::internal::read_ros2_node_names(kNsInode, kDomainId);

  ASSERT_TRUE(names.has_value());
  EXPECT_EQ(*names, (std::vector<std::string>{"/talker"}));
}

TEST_F(Ros2NodeRegistryReaderTest, SkipsMalformedLines)
{
  write_file("no_tab_here\n/\ttalker\n\t\n");

  const auto names = agnocast::internal::read_ros2_node_names(kNsInode, kDomainId);

  ASSERT_TRUE(names.has_value());
  EXPECT_EQ(*names, (std::vector<std::string>{"/talker"}));
}

// The writer replaces the file atomically, so a torn tail should not happen; dropping it keeps a
// hand-truncated or otherwise damaged file from inventing a node name.
TEST_F(Ros2NodeRegistryReaderTest, DropsUnterminatedTail)
{
  write_file("/\ttalker\n/\tlist");

  const auto names = agnocast::internal::read_ros2_node_names(kNsInode, kDomainId);

  ASSERT_TRUE(names.has_value());
  EXPECT_EQ(*names, (std::vector<std::string>{"/talker"}));
}

// An agent killed without unlinking its file leaves a snapshot behind; once it stops being
// refreshed the names in it may be long gone, so the reader disowns it.
TEST_F(Ros2NodeRegistryReaderTest, ReturnsNulloptWhenStale)
{
  write_file("/\ttalker\n");
  age_file(static_cast<int>(agnocast::internal::kStaleAfter.count()) + 1);

  EXPECT_FALSE(agnocast::internal::read_ros2_node_names(kNsInode, kDomainId).has_value());
}

}  // namespace
