#include <Simulator/CommandLineParser.h>
#include <Simulator/SimulationReportWriter.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

namespace simulator
{
namespace
{

class TemporaryDirectory final
{
public:
    TemporaryDirectory()
        : path_(std::filesystem::temp_directory_path() /
                ("assignment3_simulator_tests_" +
                 std::to_string(std::chrono::steady_clock::now()
                                    .time_since_epoch()
                                    .count())))
    {
        std::filesystem::create_directories(path_);
    }

    ~TemporaryDirectory()
    {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }

    TemporaryDirectory(const TemporaryDirectory&) = delete;
    TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;

    [[nodiscard]] const std::filesystem::path& path() const noexcept
    {
        return path_;
    }

private:
    std::filesystem::path path_;
};

TEST(CommandLineParserTest, ReportsEveryUnsupportedAndMissingArgument)
{
    const char* arguments[] = {
        "simulator_212200943", "-comparative", "unsupported_a=1",
        "unsupported_b=2"};
    try {
        static_cast<void>(CommandLineParser::parse(4, arguments));
        FAIL() << "Parsing should have failed.";
    } catch (const CommandLineError& error) {
        const std::string message = error.what();
        EXPECT_NE(message.find("unsupported_a"), std::string::npos);
        EXPECT_NE(message.find("unsupported_b"), std::string::npos);
        EXPECT_NE(message.find("simulation"), std::string::npos);
        EXPECT_NE(message.find("mission_control_folder"), std::string::npos);
        EXPECT_NE(message.find("algorithm"), std::string::npos);
    }
}

TEST(SimulationReportWriterTest, GroupsOnlyExactMatchingResults)
{
    const std::vector<PluginResultSummary> input{
        {"c.so", 20.0, 5}, {"a.so", 10.0, 7}, {"b.so", 10.0, 7},
        {"d.so", 10.0, 8}};
    const auto groups = SimulationReportWriter::groupComparativeResults(input);
    ASSERT_EQ(groups.size(), 3U);
    EXPECT_EQ(groups[0].mission_controls,
              (std::vector<std::string>{"a.so", "b.so"}));
    EXPECT_DOUBLE_EQ(groups[0].total_score, 10.0);
    EXPECT_EQ(groups[0].total_steps, 7U);
}

TEST(SimulationReportWriterTest, RanksByScoreThenStepsThenFilename)
{
    const auto ranked = SimulationReportWriter::rankCompetitionResults(
        {{"c.so", 9.0, 4}, {"b.so", 10.0, 5}, {"a.so", 10.0, 5},
         {"d.so", 10.0, 4}});
    ASSERT_EQ(ranked.size(), 4U);
    EXPECT_EQ(ranked[0].plugin_file, "d.so");
    EXPECT_EQ(ranked[1].plugin_file, "a.so");
    EXPECT_EQ(ranked[2].plugin_file, "b.so");
    EXPECT_EQ(ranked[3].plugin_file, "c.so");
}

TEST(SimulationReportWriterTest, WritesRequiredYamlSchemas)
{
    TemporaryDirectory temporary;
    const auto comparative_file = temporary.path() / "comparative.yaml";
    const auto competitive_file = temporary.path() / "competitive.yaml";
    const auto detailed_file = temporary.path() / "detailed.yaml";

    SimulationReportWriter::writeComparativeReport(
        {"composition.yaml", "controls", "2026-01-01T00:00:00Z",
         {{"control.so", 1.0, 2}}, {}},
        comparative_file);
    SimulationReportWriter::writeCompetitionReport(
        {"composition.yaml", "control.so", "2026-01-01T00:00:00Z",
         {{"algorithm.so", 1.0, 2}}, {}},
        competitive_file);
    types::SimulationManagerReport detailed;
    detailed.composition_file = "composition.yaml";
    detailed.generated_at_utc = "2026-01-01T00:00:00Z";
    detailed.metric = "output_map_accuracy";
    detailed.score_range = {0.0, 100.0};
    detailed.error_score = -1;
    SimulationReportWriter::writeDetailedReport(
        detailed, {}, {}, "composition.yaml", detailed_file);

    const YAML::Node comparative = YAML::LoadFile(comparative_file.string());
    ASSERT_TRUE(comparative["comparative_report"]);
    ASSERT_TRUE(comparative["comparative_report"]["results_summary"]);
    EXPECT_EQ(comparative["comparative_report"]["results_summary"][0]
                  ["same_results"][0]
                      .as<std::string>(),
              "control.so");

    const YAML::Node competitive = YAML::LoadFile(competitive_file.string());
    ASSERT_TRUE(competitive["competitive_report"]);
    ASSERT_TRUE(competitive["competitive_report"]["results_summary"]);
    EXPECT_EQ(competitive["competitive_report"]["results_summary"][0]
                  ["algorithm"]
                      .as<std::string>(),
              "algorithm.so");

    const YAML::Node detailed_yaml = YAML::LoadFile(detailed_file.string());
    ASSERT_TRUE(detailed_yaml["score_report"]);
    EXPECT_EQ(detailed_yaml["score_report"]["summary"]["total_runs"]
                  .as<std::size_t>(),
              0U);
}

} // namespace
} // namespace simulator
