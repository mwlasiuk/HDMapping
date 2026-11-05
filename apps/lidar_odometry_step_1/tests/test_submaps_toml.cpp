#include <doctest.h>

#include "../toml_io.h"

#include <filesystem>
#include <fstream>

namespace
{
    // A parameter file under the OS temp dir, removed when the test case ends.
    class TempToml
    {
    public:
        explicit TempToml(const std::string& content = "")
            : m_path((std::filesystem::temp_directory_path() /
                      ("submaps_toml_test_" + std::to_string(reinterpret_cast<uintptr_t>(this)) + ".toml"))
                         .string())
        {
            if (!content.empty())
            {
                std::ofstream f(m_path);
                f << content;
            }
        }

        ~TempToml()
        {
            std::filesystem::remove(m_path);
        }

        const std::string& path() const
        {
            return m_path;
        }

    private:
        std::string m_path;
    };

    void check_resolution(const SubmapParams& s, double x, double y, double z)
    {
        CHECK(s.grid.resolution_X == doctest::Approx(x));
        CHECK(s.grid.resolution_Y == doctest::Approx(y));
        CHECK(s.grid.resolution_Z == doctest::Approx(z));
    }
} // namespace

TEST_CASE("default submaps: two levels, the former indoor and outdoor maps")
{
    LidarOdometryParams params{};
    CHECK(params.number_of_submaps == 2);
    check_resolution(params.submaps[0], 0.1, 0.1, 0.1);
    check_resolution(params.submaps[1], 0.3, 0.3, 0.3);
    CHECK(params.submaps[0].min_range == 0.0);
    CHECK(params.submaps[1].min_range == 5.0);
    for (const SubmapParams& s : params.submaps)
        CHECK(s.weight == 1.0);
    CHECK(active_submaps(params) == 2);

    params.ablation_study_use_hierarchical_rgd = false;
    CHECK(active_submaps(params) == 1);

    params.ablation_study_use_hierarchical_rgd = true;
    params.number_of_submaps = 20;
    CHECK(active_submaps(params) == kMaxSubmaps);
    params.number_of_submaps = 0;
    CHECK(active_submaps(params) == kMinSubmaps);
}

TEST_CASE("a parameter file without [submaps] sets levels 0 and 1 from its indoor/outdoor grids")
{
    TempToml file(R"(
[in_out_params_indoor]
resolution_X = 0.2
resolution_Y = 0.25
resolution_Z = 0.15

[in_out_params_outdoor]
resolution_X = 0.6
resolution_Y = 0.7
resolution_Z = 0.8
)");
    LidarOdometryParams params{};
    TomlIO toml_io;
    REQUIRE(toml_io.LoadParametersFromTomlFile(file.path(), params));
    CHECK(params.number_of_submaps == 2);
    check_resolution(params.submaps[0], 0.2, 0.25, 0.15);
    check_resolution(params.submaps[1], 0.6, 0.7, 0.8);
    CHECK(params.submaps[1].min_range == 5.0);
}

TEST_CASE("[submaps] round trip, with indoor/outdoor kept in step with levels 0 and 1")
{
    LidarOdometryParams saved{};
    saved.number_of_submaps = 5;
    for (int level = 0; level < kMaxSubmaps; ++level)
    {
        saved.submaps[level].grid.resolution_X = 0.1 * (level + 1);
        saved.submaps[level].grid.resolution_Y = 0.1 * (level + 1) + 0.01;
        saved.submaps[level].grid.resolution_Z = 0.1 * (level + 1) + 0.02;
        saved.submaps[level].min_range = level;
        saved.submaps[level].weight = 0.5 + level;
    }

    TempToml file;
    TomlIO toml_io;
    REQUIRE(toml_io.SaveParametersToTomlFile(file.path(), saved));

    LidarOdometryParams loaded{};
    REQUIRE(toml_io.LoadParametersFromTomlFile(file.path(), loaded));
    CHECK(loaded.number_of_submaps == 5);
    for (int level = 0; level < kMaxSubmaps; ++level)
    {
        CAPTURE(level);
        check_resolution(loaded.submaps[level], 0.1 * (level + 1), 0.1 * (level + 1) + 0.01, 0.1 * (level + 1) + 0.02);
        CHECK(loaded.submaps[level].min_range == doctest::Approx(level));
        CHECK(loaded.submaps[level].weight == doctest::Approx(0.5 + level));
    }

    // a build without [submaps] reads levels 0 and 1 from these
    const toml::table tbl = toml::parse_file(file.path());
    CHECK(tbl["in_out_params_indoor"]["resolution_X"].value<double>() == doctest::Approx(0.1));
    CHECK(tbl["in_out_params_outdoor"]["resolution_Z"].value<double>() == doctest::Approx(0.22));
}

TEST_CASE("[submaps] entries out of range or missing leave levels as they were")
{
    TempToml file(R"(
[submaps]
number_of_submaps = 12
resolutions = [ [ 0.2, 0.2, 0.2 ], [ 0.0, 0.5, 0.5 ], [ 1, 1, 1 ] ]
min_ranges = [ 0.0, 3.0 ]
)");
    LidarOdometryParams params{};
    TomlIO toml_io;
    REQUIRE(toml_io.LoadParametersFromTomlFile(file.path(), params));
    CHECK(params.number_of_submaps == kMaxSubmaps);
    check_resolution(params.submaps[0], 0.2, 0.2, 0.2);
    check_resolution(params.submaps[1], 0.3, 0.3, 0.3); // 0.0 rejected: the default stays
    check_resolution(params.submaps[2], 1.0, 1.0, 1.0); // integers are accepted
    check_resolution(params.submaps[3], 0.75, 0.75, 0.75);
    CHECK(params.submaps[1].min_range == 3.0);
    CHECK(params.submaps[2].min_range == 5.0);
    CHECK(params.submaps[2].weight == 1.0);
}
